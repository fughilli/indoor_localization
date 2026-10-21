"""Control a swarm through the node on USB (docs/swarm-network.md).

  # fresh board: bootloader + partition table + loader + node (main)
  swarm_ctl.py flash --port P --loader loader_esp32.bin --main node_esp32.bin
  # provision the swarm's settings key (once per board, over USB)
  swarm_ctl.py key --port P

  swarm_ctl.py settings --port P                 # the attached node
  swarm_ctl.py settings --port P --node 7408     # another node, over the mesh
  swarm_ctl.py set --port P range_sigma=0.08     # attached node only
  swarm_ctl.py set --port P --node 7408 antenna_delay=16380
  swarm_ctl.py set --port P --swarm coop.mean_only_range_damping=20
  swarm_ctl.py clear --port P --node 7408 antenna_delay   (or --all)

  # sign an image with the next version and spread it through the swarm
  swarm_ctl.py push --port P node_esp32.bin
  swarm_ctl.py status --port P

Keys live in ~/.config/swarm (fw_signing_key.pem: ECDSA P-256 private key;
settings_hmac.key: 32-byte HMAC key), with a little state (override
sequence numbers, last firmware version) in state.json. Runs on the machine
the board is plugged into (pyserial; `cryptography` for push). Doesn't reset
the board on open. Stop swarm_view first: one program per serial port.
"""

import argparse
import json
import os
import subprocess
import sys
import time

import swarm_wire as w

CONFIG = os.path.expanduser("~/.config/swarm")
HMAC_KEY = os.path.join(CONFIG, "settings_hmac.key")
FW_KEY = os.path.join(CONFIG, "fw_signing_key.pem")
STATE = os.path.join(CONFIG, "state.json")
BAUD = 460800


# ---- serial ------------------------------------------------------------------

class Node:
    """Newline-JSON link to the node on USB."""

    def __init__(self, port):
        import serial

        # Leave DTR/RTS as the OS sets them: deasserting them passes through
        # the auto-reset circuit's "reset" state (see capture.py).
        self.s = serial.Serial(port, BAUD, timeout=0.05)
        self.buf = b""
        # The first bytes written right after opening sometimes never reach
        # the board: let the port settle first.
        time.sleep(0.3)
        self.s.reset_input_buffer()

    def send(self, obj):
        self.s.write((json.dumps(obj, separators=(",", ":")) + "\n").encode())

    def records(self, seconds):
        """Yield records for up to `seconds` (forever if None)."""
        end = None if seconds is None else time.time() + seconds
        while end is None or time.time() < end:
            self.buf += self.s.read(4096)
            while b"\n" in self.buf:
                line, self.buf = self.buf.split(b"\n", 1)
                # After a reset the boot ROM's output (no trailing newline)
                # prefixes the first JSON line: parse from the first brace.
                line = line[line.find(b"{"):].strip() if b"{" in line else b""
                if line.startswith(b"{"):
                    try:
                        yield json.loads(line)
                    except ValueError:
                        pass

    def call(self, obj, cmd=None, seconds=3.0, retry=True):
        """Send a command; return its ack record (or raise). Commands are
        idempotent on the node, so a lost line is simply sent again."""
        cmd = cmd or obj["cmd"]
        for attempt in range(2 if retry else 1):
            self.send(obj)
            for r in self.records(seconds):
                if r.get("type") == "ack" and r.get("cmd") in (cmd, "unknown"):
                    if not r.get("ok"):
                        raise SystemExit("%s failed: %s" % (cmd, r))
                    return r
        raise SystemExit("no answer to %s (is swarm_view holding the port?)" % cmd)

    def settings(self):
        """The attached node's registry with current values:
        (header, [setting records])."""
        for attempt in range(2):
            self.send({"cmd": "settings"})
            head, out = None, []
            for r in self.records(3.0):
                t = r.get("type")
                if t == "settings_begin":
                    head, out = r, []
                elif t == "setting" and head is not None:
                    out.append(r)
                elif t == "settings_end" and head is not None:
                    return head, out
        raise SystemExit("no settings listing")


# ---- state & keys -------------------------------------------------------------

def load_state():
    try:
        with open(STATE) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def save_state(st):
    os.makedirs(CONFIG, exist_ok=True)
    tmp = STATE + ".tmp"
    with open(tmp, "w") as f:
        json.dump(st, f, indent=1)
    os.replace(tmp, STATE)


def hmac_key():
    try:
        with open(HMAC_KEY, "rb") as f:
            k = f.read()
    except OSError:
        raise SystemExit("no settings key at %s" % HMAC_KEY)
    if len(k) != 32:
        raise SystemExit("%s must hold 32 bytes" % HMAC_KEY)
    return k


def next_counter(st, name, at_least=0):
    """Increasing u32: wall-clock seconds, or last + 1 if that's not newer."""
    v = max(int(time.time()), st.get(name, 0) + 1, at_least + 1)
    st[name] = v
    return v


# ---- settings -----------------------------------------------------------------

def registry_by_name(regs):
    return {r["name"]: r for r in regs}


def parse_assignments(items, by_name):
    out = []
    for item in items:
        if "=" not in item:
            raise SystemExit("expected name=value, got %r" % item)
        name, value = item.split("=", 1)
        r = by_name.get(name)
        if not r:
            raise SystemExit("unknown setting %r" % name)
        raw = w.encode_value(r["kind"], value)
        v = w.decode_value(r["kind"], raw)
        if r["kind"] != "bool" and not (r["min"] <= v <= r["max"]):
            raise SystemExit("%s=%s outside [%g, %g]" % (name, value, r["min"], r["max"]))
        out.append((r, raw))
    return out


def remote_settings(node, dest, regs):
    """A node's layers over the mesh: {name: (source, value)} + header."""
    by_id = {r["id"]: r for r in regs}
    parts, header = {}, None
    # The GET and its report are unacknowledged mesh packets: ask again if
    # the answer doesn't come.
    for attempt in range(3):
        node.call({"cmd": "send", "dest": dest, "port": w.PORT_SETTINGS_GET, "hex": ""}, "send")
        total = None
        for r in node.records(4.0):
            if r.get("type") == "mesh" and r.get("origin") == dest and r.get("port") == w.PORT_SETTINGS_REPORT:
                part, total, gen, seq, entries = w.parse_report(bytes.fromhex(r["hex"]))
                header = {"generation": gen, "override_seq": seq, "hops": r.get("hops"), "rssi": r.get("rssi")}
                parts[part] = entries
                if len(parts) == total:
                    break
        if total is not None and len(parts) == total:
            break
    if header is None:
        raise SystemExit("no report from node %d" % dest)
    values = {}
    for entries in parts.values():
        for key, source, raw in entries:
            reg = by_id.get(key)
            name = reg["name"] if reg else "#%d" % key
            v = w.decode_value(reg["kind"], raw) if reg else raw
            values.setdefault(name, {})["override" if source == w.SOURCE_OVERRIDE else "swarm"] = v
    return header, values


def cmd_settings(a):
    node = Node(a.port)
    head, regs = node.settings()
    if a.node is None:
        print("node %d  swarm generation %d  override seq %d  key %s" %
              (head["id"], head["generation"], head["override_seq"], "yes" if head["key"] else "NO"))
        for r in regs:
            if a.changed and r["source"] == "default":
                continue
            print("  %-34s %-12s %-8s (default %s)" % (r["name"], r["value"], r["source"], r["default"]))
        return
    header, values = remote_settings(node, a.node, regs)
    print("node %d  swarm generation %d  override seq %d  (%s hops, %.0f dBm)" %
          (a.node, header["generation"], header["override_seq"], header["hops"], header["rssi"]))
    for name in sorted(values):
        v = values[name]
        print("  %-34s %s" % (name, "  ".join("%s=%s" % kv for kv in sorted(v.items()))))
    if not values:
        print("  (all defaults)")


def push_swarm_layer(node, head, regs, changes, a):
    """Rebuild the swarm layer from the attached node's copy + changes,
    tag it with the next generation, hand it to the node to spread."""
    layer = {}
    for r in regs:
        if r["swarm"] is not None:
            layer[r["id"]] = w.encode_value(r["kind"], r["swarm"])
    for reg, raw in changes:
        if raw is None:
            layer.pop(reg["id"], None)
        else:
            layer[reg["id"]] = raw
    st = load_state()
    gen = next_counter(st, "generation", head["generation"])
    bundle = w.make_bundle(layer, gen, hmac_key())
    node.call({"cmd": "bundle", "hex": bundle.hex()})
    save_state(st)
    print("swarm generation %d: %d settings, spreading through the swarm" % (gen, len(layer)))


def send_override(node, dest, kind, entries):
    st = load_state()
    seqs = st.setdefault("override_seq", {})
    seq = max(int(time.time()), seqs.get(str(dest), 0) + 1)
    seqs[str(dest)] = seq
    payload = w.make_override(dest, kind, seq, entries, hmac_key())
    node.call({"cmd": "send", "dest": dest, "port": w.PORT_OVERRIDE, "ack": True, "hex": payload.hex()}, "send")
    save_state(st)


def cmd_set(a):
    node = Node(a.port)
    head, regs = node.settings()
    changes = parse_assignments(a.assignments, registry_by_name(regs))
    if a.swarm:
        return push_swarm_layer(node, head, regs, changes, a)
    if a.node is None:
        for reg, raw in changes:
            node.call({"cmd": "set", "name": reg["name"], "value": w.decode_value(reg["kind"], raw)})
        print("set on node %d" % head["id"])
        return
    for i in range(0, len(changes), w.CMD_MAX_ENTRIES):
        send_override(node, a.node, w.CMD_SET, [(r["id"], raw) for r, raw in changes[i:i + w.CMD_MAX_ENTRIES]])
    time.sleep(1.0)
    _, values = remote_settings(node, a.node, regs)
    for reg, raw in changes:
        got = values.get(reg["name"], {}).get("override")
        print("  node %d %s = %s%s" % (a.node, reg["name"], got,
                                       "" if got is not None else "  (not confirmed yet; retry `settings --node`)"))


def cmd_clear(a):
    node = Node(a.port)
    head, regs = node.settings()
    by_name = registry_by_name(regs)
    names = [] if a.all else a.names
    for n in names:
        if n not in by_name:
            raise SystemExit("unknown setting %r" % n)
    if a.swarm:
        changes = [(by_name[n], None) for n in names] if names else [(r, None) for r in regs]
        return push_swarm_layer(node, head, regs, changes, a)
    if a.node is None:
        if a.all:
            node.call({"cmd": "clear", "all": True})
        for n in names:
            node.call({"cmd": "clear", "name": n})
        print("cleared on node %d" % head["id"])
        return
    entries = [(by_name[n]["id"], 0) for n in names]
    for i in range(0, max(len(entries), 1), w.CMD_MAX_ENTRIES):
        send_override(node, a.node, w.CMD_CLEAR, entries[i:i + w.CMD_MAX_ENTRIES])
    print("cleared on node %d" % a.node)


def cmd_key(a):
    node = Node(a.port)
    if a.remove:
        node.call({"cmd": "key", "remove": True})
        print("key removed")
        return
    if not os.path.exists(HMAC_KEY):
        os.makedirs(CONFIG, mode=0o700, exist_ok=True)
        fd = os.open(HMAC_KEY, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        os.write(fd, os.urandom(32))
        os.close(fd)
    node.call({"cmd": "key", "hex": hmac_key().hex()})
    print("settings key provisioned")


# ---- firmware -----------------------------------------------------------------

def cmd_push(a):
    with open(a.image, "rb") as f:
        image = f.read()
    with open(FW_KEY, "rb") as f:
        pem = f.read()
    node = Node(a.port)
    st = load_state()
    version = a.version or st.get("fw_version", 0) + 1
    manifest = w.sign_manifest(version, image, pem)
    # Re-offering the manifest that's already staged resumes a push.
    start = 0
    status = node_status(node)
    if status and status["fw"]["staged"] == version and status["fw"]["status"] == "receiving":
        start = status["fw"]["pages"]
        # A resumed push must carry the identical image (ECDSA signatures
        # differ per signing, but the node matches the whole manifest).
        manifest = bytes.fromhex(st.get("fw_manifest_hex", "")) or manifest
        print("resuming at page %d" % start)
    node.call({"cmd": "fw_manifest", "hex": manifest.hex()})
    st["fw_manifest_hex"] = manifest.hex()
    save_state(st)
    pages = (len(image) + w.PAGE_SIZE - 1) // w.PAGE_SIZE
    t0 = time.time()
    for page in range(start, pages):
        data = image[page * w.PAGE_SIZE:(page + 1) * w.PAGE_SIZE]
        node.call({"cmd": "fw_page", "page": page, "hex": data.hex()}, seconds=10.0)
        sys.stdout.write("\r  staged %d/%d pages (%.0f s)" % (page + 1, pages, time.time() - t0))
        sys.stdout.flush()
    print()
    st["fw_version"] = version
    save_state(st)
    print("version %d (%d bytes) staged on the attached node; it installs it and the swarm fetches it" %
          (version, len(image)))
    if a.wait:
        watch(node, a.wait)


def node_status(node):
    node.send({"cmd": "net"})
    for r in node.records(3.0):
        if r.get("type") == "net":
            return r
    return None


def watch(node, seconds):
    for r in node.records(seconds):
        if r.get("type") in ("net", "installing", "boot", "installed", "marked_valid"):
            if r.get("type") == "net":
                fw = r["fw"]
                print("  %s %s v%d: %s %d/%d" % (r.get("app"), r.get("id"), fw["running"], fw["status"],
                                                 fw["pages"], fw["total"]))
            else:
                print("  ", r)


def cmd_status(a):
    node = Node(a.port)
    for r in node.records(a.seconds):
        if r.get("type") == "net":
            print(json.dumps(r))


def cmd_flash(a):
    """Bootloader, partition table, loader, main; otadata erased (the
    loader boots first and hands over to main)."""
    esptool = [sys.executable, "-m", "esptool", "--chip", "esp32", "--port", a.port, "--baud", "460800"]
    subprocess.check_call(esptool + ["erase_region", "0xe000", "0x2000"])
    subprocess.check_call(esptool + ["write_flash", "--flash_mode", "keep", "--flash_freq", "keep",
                                     "--flash_size", "keep",
                                     "0x1000", a.bootloader, "0x8000", a.partitions,
                                     "0x10000", a.loader, "0xB0000", a.main])


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    def add(name, fn, **kw):
        p = sub.add_parser(name, **kw)
        p.add_argument("--port", required=True)
        p.set_defaults(fn=fn)
        return p

    p = add("settings", cmd_settings, help="list settings")
    p.add_argument("--node", type=int)
    p.add_argument("--changed", action="store_true", help="only non-default")
    p = add("set", cmd_set, help="set settings (name=value ...)")
    g = p.add_mutually_exclusive_group()
    g.add_argument("--node", type=int, help="another node, over the mesh")
    g.add_argument("--swarm", action="store_true", help="every node (swarm layer)")
    p.add_argument("assignments", nargs="+")
    p = add("clear", cmd_clear, help="clear overrides (or swarm values)")
    g = p.add_mutually_exclusive_group()
    g.add_argument("--node", type=int)
    g.add_argument("--swarm", action="store_true")
    p.add_argument("--all", action="store_true")
    p.add_argument("names", nargs="*")
    p = add("key", cmd_key, help="provision the settings HMAC key")
    p.add_argument("--remove", action="store_true")
    p = add("push", cmd_push, help="sign + push a firmware image")
    p.add_argument("image")
    p.add_argument("--version", type=int, help="default: last pushed + 1")
    p.add_argument("--wait", type=float, default=0, help="then watch status this many seconds")
    p = add("status", cmd_status, help="print network status lines")
    p.add_argument("--seconds", type=float, default=3)
    p = add("flash", cmd_flash, help="USB-flash the full layout")
    p.add_argument("--bootloader", required=True)
    p.add_argument("--partitions", required=True)
    p.add_argument("--loader", required=True)
    p.add_argument("--main", required=True)
    a = ap.parse_args(argv)
    a.fn(a)


if __name__ == "__main__":
    main()

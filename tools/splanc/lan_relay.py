"""TCP relay 127.0.0.1:<listen> -> <host>:<port>.

Workaround for macOS Local Network privacy blocking a tool (here Homebrew's
node, which runs splanc-agent) from reaching a LAN device. Run it with Apple's
exempt interpreter and point the tool at the loopback port:

  /usr/bin/python3 -I tools/splanc/lan_relay.py 18443 192.168.68.70 443
  python3 tools/splanc/range_bridge.py --host 127.0.0.1:18443 --deploy

Listens on loopback only and forwards to the one address given.
"""

import socket
import sys
import threading


def pump(src, dst):
    try:
        while True:
            data = src.recv(65536)
            if not data:
                break
            dst.sendall(data)
    except OSError:
        pass
    finally:
        for s in (src, dst):
            try:
                s.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass


def main():
    listen, host, port = int(sys.argv[1]), sys.argv[2], int(sys.argv[3])
    srv = socket.create_server(("127.0.0.1", listen))
    print("relay 127.0.0.1:%d -> %s:%d" % (listen, host, port), flush=True)
    while True:
        client, _ = srv.accept()
        try:
            upstream = socket.create_connection((host, port), timeout=5)
            upstream.settimeout(None)
        except OSError as e:
            print("upstream %s:%d: %s" % (host, port, e), flush=True)
            client.close()
            continue
        threading.Thread(target=pump, args=(client, upstream), daemon=True).start()
        threading.Thread(target=pump, args=(upstream, client), daemon=True).start()


if __name__ == "__main__":
    main()

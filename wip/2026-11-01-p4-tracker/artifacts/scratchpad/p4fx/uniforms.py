import asyncio, base64, struct, subprocess, sys
import websockets
from server import proto_wire

async def rpc(ws, msg, want):
    await ws.send(proto_wire.encode_client(msg))
    while True:
        r = proto_wire.decode_server(await asyncio.wait_for(ws.recv(), 5))
        if r["type"] == want: return r
        if r["type"] == "error": raise SystemExit(f"error: {r}")

async def main(url, fxc, src):
    out = src + ".fxb"
    subprocess.run([fxc, src, out], check=True)
    fxb = open(out, "rb").read()
    async with websockets.connect(url, max_size=1 << 20) as ws:
        await rpc(ws, {"type": "hello", "client": "p4-tracker-check", "app_version": "1"}, "welcome")
        await rpc(ws, {"type": "submit_effect", "effect_id": "p4_tracker_check",
                       "fxb": base64.b64encode(fxb).decode(), "activate": True}, "result_ready")
        for _ in range(5):
            await asyncio.sleep(1.0)
            r = await rpc(ws, {"type": "get_fx_snapshot", "section": 2, "deck": 0, "offset": 0, "maxLen": 768}, "fx_debug")
            c = r["chunk"]; data = base64.b64decode(c.get("data", ""))
            v = struct.unpack("<%df" % (len(data) // 4), data)
            print(f"frame {c.get('frame')}: uniform slots {[round(x, 4) for x in v[:8]]}", flush=True)

asyncio.run(main(*sys.argv[1:]))

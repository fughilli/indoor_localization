import serial, sys, time
s = serial.Serial('/dev/cu.usbmodem5CF71536071', 115200, timeout=0.2)
dur = float(sys.argv[1]) if len(sys.argv) > 1 else 15
if len(sys.argv) > 2 and sys.argv[2] == 'reset':
    s.dtr = False; s.rts = True; time.sleep(0.1); s.rts = False
end = time.time() + dur
while time.time() < end:
    d = s.read(4096)
    if d: sys.stdout.write(d.decode('utf-8', 'replace')); sys.stdout.flush()

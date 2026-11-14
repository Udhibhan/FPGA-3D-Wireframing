#!/usr/bin/env python3
"""USB bridge: STM32 drone <-> laptop <-> GreyMecha Command Centre.

Use this when the boards are NOT wired together (STM32: GM_USB_BRIDGE 1,
GreyMecha code.py: LINK = "usb").

- Lines from the drone starting with '$' go to the GreyMecha.
- Every other drone line is printed here, so this window is also your terminal.
- 'CMD:x' lines from the GreyMecha are sent to the drone as the byte x.
- Type d, s or p then Enter to send a double press, single press or ping yourself.

Usage: python bridge.py /dev/ttyACM0 /dev/ttyACM1     (drone port first)
"""
import sys
import threading

import serial

DRONE_PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
GM_PORT = sys.argv[2] if len(sys.argv) > 2 else "/dev/ttyACM1"

drone = serial.Serial(DRONE_PORT, 115200, timeout=0.1)
gm = serial.Serial(GM_PORT, 115200, timeout=0.1)


def whole_lines(port):
    """Yield complete lines only. readline() can return half a line when its
    timeout hits mid-line, so keep the start until the newline arrives."""
    part = b""
    while True:
        chunk = port.readline()
        if not chunk:
            continue
        part += chunk
        if part.endswith(b"\n"):
            yield part
            part = b""


def drone_to_gm():
    for line in whole_lines(drone):
        if line.startswith(b"$"):
            gm.write(line)
        else:
            sys.stdout.write(line.decode(errors="replace"))
            sys.stdout.flush()


def gm_to_drone():
    for line in whole_lines(gm):
        text = line.decode(errors="replace").strip()
        if text.startswith("CMD:") and len(text) >= 5:
            drone.write(text[4].encode())
        elif text:
            print("[Command Centre]", text)


threading.Thread(target=drone_to_gm, daemon=True).start()
threading.Thread(target=gm_to_drone, daemon=True).start()
print(f"Bridging drone {DRONE_PORT} <-> Command Centre {GM_PORT}. Ctrl-C to stop.")

try:
    for typed in sys.stdin:
        c = typed.strip().upper()[:1]
        if c in ("D", "S", "P"):
            drone.write(c.encode())
except KeyboardInterrupt:
    pass

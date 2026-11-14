# EE2028 Command Centre - GreyMecha/Army (RP2350 + ECP5 FPGA)
#
# The RP2350 decides WHAT to show; the FPGA draws it.
# 1. Loads our FPGA design (cc_radar.bit): radar display, LEDs and buzzer
# 2. Reads the drone's "$D" status lines (jumper wires, or USB via bridge.py)
# 3. Sends a 32-byte status packet to the FPGA over SPI, 20 times per second
# 4. Button A = remote double press (rescue / change mode)
#    Button B = remote single press (charge the laser)
# 5. Forwards the drone's attitude ("$A" lines) to the FPGA for the 3D view.
#    The FPGA's own button cross controls that view: centre = 3D on/off,
#    left/right = turn the aim, up = aim offset back to 0.
# 6. Zombie game runs on the FPGA: IR sees something -> zombie; laser shot kills
#    the nearest one in the aim (big ones take 3). The FPGA reports hits, kills,
#    misses, approaching and bites back; they are passed on to the drone.
#
# Hold B while the board starts to get the original GreyMecha menu instead.

import time
import board
import digitalio

LINK = "usb"               # "usb" = laptop bridge (bridge.py), "uart" = jumper wires
BITSTREAM = "/cc_radar.bit"
LOST_AFTER_S = 2.0         # no status line for this long = drone lost
PACKET_EVERY_S = 0.05      # status packets to the FPGA: 20 per second
STATE_NAMES = ("STANDBY", "BATTLE", "ATTACKED", "LOST")
WARN_NAMES = ("TMP", "PRS", "HUM", "ACC", "GYR", "MAG")   # bit 0..5 of the warning field

# 3D view: if the model tilts or turns the opposite way to the drone,
# change the matching sign to -1 and save (no FPGA rebuild needed)
ROLL_SIGN = 1
PITCH_SIGN = 1
YAW_SIGN = 1

# ---- Escape hatch: B held at power-on -> original menu (factory_code.py) ------
_b = digitalio.DigitalInOut(board.GP1)
_b.switch_to_input(pull=digitalio.Pull.UP)
_factory = not _b.value
_b.deinit()                  # give the pin back before anything else uses it
if _factory:
    import factory_code  # noqa: F401  (runs the factory menu, never returns)
    while True:
        pass

# ---- 1. Load the FPGA design ---------------------------------------------------
# /ccfpga is a copy of the factory hardware/fpga JTAG loader. Using the copy
# avoids the factory start-up code, which grabs the screen and button pins.
# The FPGA drives the round screen, so the RP2350 must leave GP2..GP6 alone.
import displayio
displayio.release_displays()
try:
    from ccfpga import ecp5p
except ImportError:
    print("ERROR: /ccfpga folder missing - do the 'copy the FPGA loader' step")
    raise

fpga_reset = digitalio.DigitalInOut(board.GP20)   # FPGA PROGRAMN pin
fpga_reset.switch_to_output(value=False)          # low = clear the old design
time.sleep(0.1)
fpga_reset.value = True
print("Loading FPGA radar:", BITSTREAM)
ecp5p.prog(BITSTREAM)                             # send the design over JTAG
fpga_reset.deinit()
print("FPGA radar loaded")

# ---- 2. Our pins: buttons + SPI link to the FPGA -------------------------------
# SPI1: GP10 = clock, GP11 = data, GP9 = chip select (FPGA interconnect[2], [3], [1])
import busio
try:
    btn_a = digitalio.DigitalInOut(board.GP7)       # pressed = False
    btn_a.switch_to_input(pull=digitalio.Pull.UP)
    btn_b = digitalio.DigitalInOut(board.GP1)
    btn_b.switch_to_input(pull=digitalio.Pull.UP)
    fpga_cs = digitalio.DigitalInOut(board.GP9)
    fpga_cs.switch_to_output(value=True)
    # GP8 (interconnect[0]) carries the FPGA's reply: the zombie game's events
    fpga_spi = busio.SPI(clock=board.GP10, MOSI=board.GP11, MISO=board.GP8)
except ValueError as e:
    print("ERROR:", e, "- unplug and replug the GreyMecha, then try again")
    raise
while not fpga_spi.try_lock():
    pass
fpga_spi.configure(baudrate=1_000_000, polarity=0, phase=0)

packet = bytearray(32)


def text12(s):
    """12 characters, centred, upper case (the FPGA font has no lower case)."""
    s = s.upper()[:12]
    pad = (12 - len(s)) // 2
    s = " " * pad + s + " " * (12 - pad - len(s))
    return bytes((c if 0x20 <= c < 0x60 else 0x3F) for c in s.encode())


reply = bytearray(32)


def send_to_fpga(state, charge, target, shots, heading_deg, secs, hits, line1, line2, warn=0):
    """Pack the status into 32 bytes; the FPGA checks the sync byte and checksum."""
    packet[0] = 0xA5
    packet[1] = state & 3
    packet[2] = max(0, min(100, charge))
    # bit 0 IR target, bit 1 toggles per shot, bit 2 sensor warning
    packet[3] = (target & 1) | ((shots & 1) << 1) | ((warn & 1) << 2)
    packet[4] = (heading_deg * 256 // 360) & 0xFF
    packet[5] = max(0, min(10, secs))
    packet[6] = hits & 0xFF
    packet[7:19] = text12(line1)
    packet[19:31] = text12(line2)
    check = 0
    for i in range(1, 31):
        check ^= packet[i]
    packet[31] = check
    fpga_cs.value = False
    fpga_spi.write_readinto(packet, reply)   # the FPGA answers while we send
    fpga_cs.value = True
    game_events()


# ---- Zombie game events: the FPGA decides hits by the drone's aim ------------
# reply = 0x5A, then counters: hits, kills, misses, approaching, small bites, big bites.
# Each new event is sent to the drone as one letter, which prints it and shows it on its OLED:
#   K = zombie destroyed, H = big zombie hit (still standing), M = shot missed,
#   A = zombie approaching, B = small zombie reached the drone (-1 health), G = big one (-2)
game_prev = None


def game_events():
    global game_prev
    if reply[0] != 0x5A:                 # older bitstream: no reply
        return
    now_c = bytes(reply[1:7])
    if game_prev is None:                # first answer: just remember the counts
        game_prev = now_c
        return
    d = [(now_c[i] - game_prev[i]) & 0xFF for i in range(6)]
    game_prev = now_c
    if max(d) > 20:                      # out of step (e.g. FPGA reloaded): resync quietly
        return
    hits, kills, misses, nears, bites, bigbites = d
    for letter, n in ((b"K", kills), (b"H", max(0, hits - kills)), (b"M", misses),
                      (b"A", nears), (b"B", bites), (b"G", bigbites)):
        for _ in range(n):
            send_cmd(letter)


pose = bytearray(5)


def send_pose(roll_deg, pitch_deg, yaw_deg):
    """5-byte attitude packet: 0xA6, roll, pitch, yaw (0..255 = 0..360 deg), XOR."""
    pose[0] = 0xA6
    pose[1] = int(ROLL_SIGN * roll_deg * 256 / 360) & 0xFF
    pose[2] = int(PITCH_SIGN * pitch_deg * 256 / 360) & 0xFF
    pose[3] = int(YAW_SIGN * yaw_deg * 256 / 360) & 0xFF
    pose[4] = pose[1] ^ pose[2] ^ pose[3]
    fpga_cs.value = False
    fpga_spi.write(pose)
    fpga_cs.value = True


def parse_attitude(line):
    """'$A,roll,pitch,yaw' (degrees) -> (roll, pitch, yaw), or None."""
    f = line[3:].split(b",")
    if len(f) < 3:
        return None
    try:
        return int(f[0]), int(f[1]), int(f[2])
    except ValueError:
        return None


def screen_text(s, now):
    """The two text lines for the radar screen."""
    if s is None:
        return "WAITING", "FOR DRONE"
    st = s["state"]
    if st == 0 and s["warn"]:                         # a sensor is out of range
        bad = [n for i, n in enumerate(WARN_NAMES) if s["warn"] & (1 << i)]
        return "! WARNING !", " ".join(bad) if len(bad) <= 3 else "%d SENSORS" % len(bad)
    if st == 0:
        if int(now / 2) % 2:                          # alternate every 2 s
            return "STANDBY", "P%.1f HPA" % s["press"]
        return "STANDBY", "T%.1fC H%d%%" % (s["temp"], int(s["hum"]))
    if st == 1:
        if s["hp"] is None:                           # older drone code: no health field
            return "BATTLE", "HITS %d  %d%%" % (s["hits"], s["charge"])
        t = "HP%d HIT%d %d%%" % (s["hp"], s["hits"], s["charge"])
        return "BATTLE", t if len(t) <= 12 else "HP%d H%d %d%%" % (s["hp"], s["hits"], s["charge"])
    if st == 2:
        return "ATTACKED!", "PRESS A"
    return "SIGNAL", "LOST"


# ---- 3. Link to the drone ----------------------------------------------------
if LINK == "uart":
    # GP24 = TX -> STM32 D0 (PA1), GP25 = RX <- STM32 D1 (PA0)
    uart = busio.UART(board.GP24, board.GP25, baudrate=115200,
                      timeout=0, receiver_buffer_size=512)

    def read_bytes():
        n = uart.in_waiting
        return uart.read(n) if n else b""

    def send_cmd(c):
        uart.write(c)
else:
    import sys
    import supervisor

    def read_bytes():
        n = supervisor.runtime.serial_bytes_available
        return sys.stdin.read(n).encode() if n else b""

    def send_cmd(c):
        print("CMD:" + c.decode())  # bridge.py forwards this to the drone


def parse(line):
    """'$D,state,charge,ir,shots,hits,secs,T,P,H,height,heading[,warn,hp]' -> dict, or None."""
    if not line.startswith(b"$D,"):
        return None
    f = line[3:].split(b",")
    if len(f) < 11:
        return None
    try:
        return {
            "state": int(f[0]), "charge": int(f[1]), "ir": int(f[2]),
            "shots": int(f[3]), "hits": int(f[4]), "secs": int(f[5]),
            "temp": float(f[6]), "press": float(f[7]), "hum": float(f[8]),
            "height": float(f[9]), "heading": int(f[10]),
            "warn": int(f[11]) if len(f) > 11 else 0,   # older drone code has no warning field
            "hp": int(f[12]) if len(f) > 12 else None,  # drone health (zombie game)
        }
    except ValueError:
        return None


# ---- 4. Main loop --------------------------------------------------------------
buf = b""
status = None
last_rx = time.monotonic()
lost = False
prev_state = None
prev_a = prev_b = True
next_press_ok = 0.0          # 150 ms lockout against button bounce
next_packet = 0.0

send_cmd(b"P")               # ping: the drone answers "$PONG"
print("Command Centre ready. A = double press, B = single press")
print("FPGA buttons: centre = 3D view on/off, left/right = orbit, up = reset view")

while True:
    now = time.monotonic()

    # Read whatever arrived and handle complete lines
    data = read_bytes()
    if data:
        buf += data
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            line = line.strip()
            s = parse(line)
            if s:
                status = s
                last_rx = now
                lost = False
            elif line.startswith(b"$A,"):
                att = parse_attitude(line)
                if att:
                    send_pose(-att[1], att[0], att[2])   # board X axis = drone nose
            elif line.startswith(b"$PONG"):
                print("Drone answered the ping: link OK")
            elif line.startswith(b"$HELLO"):
                print("Drone powered on")
        if len(buf) > 256:   # junk without newlines: start again
            buf = b""

    # Silence for 2 s = the drone is gone
    if status and not lost and now - last_rx > LOST_AFTER_S:
        lost = True

    # Status packet to the FPGA, 20 times per second
    if now >= next_packet:
        next_packet = now + PACKET_EVERY_S
        shown = None
        if status:
            shown = dict(status)
            if lost:
                shown["state"] = 3
        line1, line2 = screen_text(shown, now)
        if shown:
            send_to_fpga(shown["state"], shown["charge"], shown["ir"], shown["shots"],
                         shown["heading"], shown["secs"], shown["hits"], line1, line2,
                         1 if (shown["state"] == 0 and shown["warn"]) else 0)
            if shown["state"] != prev_state:
                prev_state = shown["state"]
                print("Drone:", STATE_NAMES[prev_state & 3])
        else:
            send_to_fpga(3, 0, 0, 0, 0, 0, 0, line1, line2)

    # Buttons: act on the moment of pressing
    a, b = btn_a.value, btn_b.value
    if now >= next_press_ok:
        if prev_a and not a:
            send_cmd(b"D")
            print("Sent remote double press")
            next_press_ok = now + 0.15
        elif prev_b and not b:
            send_cmd(b"S")
            print("Sent remote single press")
            next_press_ok = now + 0.15
    prev_a, prev_b = a, b

    time.sleep(0.005)

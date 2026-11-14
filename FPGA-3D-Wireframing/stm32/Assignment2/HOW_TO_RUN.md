# EE2028 Assignment 2 - How to run on a fresh PC

Everything needed is in this zip. Nothing else from our laptops is required.

| Folder | What it is |
|---|---|
| `Assignment2/` | STM32CubeIDE project. All drivers are inside, with relative paths. Our code: `Core/Src/main.c` |
| `Assignment2/CommandCentre/` | Enhancement: GreyMecha Command Centre (RP2350 + ECP5 FPGA). CubeIDE does not compile this folder. |
| `CommandCentre/GreyMecha_CIRCUITPY/` | Files to copy onto the GreyMecha's CIRCUITPY drive |
| `CommandCentre/PC_bridge/` | `bridge.py` (links drone and Command Centre over USB) + pyserial installer |
| `CommandCentre/FPGA_source/` | Verilog source of the FPGA design, testbenches, and how to rebuild it |

You need: STM32CubeIDE, and Python 3 for the Command Centre bridge.

---

## A. STM32 drone (all compulsory features) - about 3 minutes

1. STM32CubeIDE -> **File -> Import -> General -> Existing Projects into Workspace -> Next**
2. Choose **Select archive file -> Browse** -> pick this zip -> make sure `Assignment2` is ticked -> **Finish**
3. Select the project -> **Project -> Build Project** (hammer). Expect 0 errors.
4. Plug in the B-L4S5I-IOT01A (ST-LINK USB port) -> **Run -> Run As -> STM32 C/C++ Application -> OK**
5. Terminal: use the bridge in part C (it is the terminal). Without the Command Centre, any terminal on the
   **STLink Virtual COM Port, 115200 baud, 8N1** works. Lines starting with `$` are machine data for the
   Command Centre and can be ignored (set `GM_USB_BRIDGE` to `0` on line 29 of `main.c` to remove them).

Optional hardware on the Grove Base Shield (the code runs without them and says so):
- Grove OLED 128x64 -> **I2C** port
- Grove IR reflective sensor -> **D2** port

---

## B. GreyMecha Command Centre - about 1 minute

1. Plug the GreyMecha into the PC with USB. A drive called **CIRCUITPY** appears.
2. Copy **everything inside** `CommandCentre/GreyMecha_CIRCUITPY/` to the root of CIRCUITPY
   (`code.py`, `cc_radar.bit`, `cc_radar_v1.bit`, the `ccfpga` folder). Replace `code.py` when asked.
   Leave the board's own factory files alone.
3. The board restarts by itself. The round screen shows the radar with **NO LINK** until part C runs.
4. Close Mu, Thonny or any serial monitor that is using the GreyMecha's port.

---

## C. Bridge (laptop links the two boards)

Windows (use `python3` instead of `py` on macOS / Linux):

```
cd <where you unzipped>\Assignment2\CommandCentre\PC_bridge
py -m pip install pyserial
py -m serial.tools.list_ports -v
py bridge.py COM5 COM7
```

- No internet? Use `py -m pip install pyserial-3.5-py2.py3-none-any.whl` (it is in the same folder).
- In the port list: **STLink Virtual COM Port** = drone, **USB Serial Device** (CircuitPython) = GreyMecha.
- Put the **drone port first**. macOS / Linux ports look like `/dev/tty.usbmodem...` or `/dev/ttyACM0`.
- This window is now the drone's terminal. Type `d`, `s` or `p` then Enter for a double press,
  single press or link check.
- Start the bridge **after** flashing the STM32 (flashing resets its port). Ctrl-C to stop.

---

## Controls

| Control | Action |
|---|---|
| STM32 blue button | single press = charge laser (+30%), double press = change mode / rescue |
| GreyMecha button A / B | remote double press / remote single press |
| GreyMecha: hold B while powering on | original GreyMecha menu instead of the Command Centre |
| FPGA button cross: centre | radar <-> 3D view |
| FPGA button cross: left / right / up | turn aim / turn aim / reset aim |
| IR sensor sees an object (in BATTLE) | a zombie spawns; turn the drone (yaw) to aim, then fire |
| Flip the drone upside down | "The Last of EE2028" (attacked, 10 s rescue window) |

---

## If something goes wrong

| Problem | Fix |
|---|---|
| Round screen says **NO LINK** | Bridge not running, ports in the wrong order, or another program holds a port |
| `could not open port` | Close TeraTerm / PuTTY / CubeIDE console on that port: one program per port |
| GreyMecha prints `/ccfpga folder missing` | Copy the `ccfpga` folder onto CIRCUITPY (part B step 2) |
| Need the simpler radar-only design | In `code.py` on CIRCUITPY, set `BITSTREAM = "/cc_radar_v1.bit"` and save |
| Bridge stopped after re-flashing | Normal (the port reset). Run `py bridge.py ...` again |

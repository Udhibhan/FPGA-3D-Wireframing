# EE2028 Command Centre - FPGA radar display

The FPGA draws the GreyMecha's 240x240 round screen pixel by pixel in hardware
(about 40 frames per second, no frame buffer), and drives the 8 LEDs and buzzer.
The RP2350 sends it a 32-byte status packet over SPI 20 times per second.

**3D view.** Press the **centre button of the FPGA's button cross** to swap the
radar for a live 3D wireframe of the drone with a gun turret: it tilts with the
STM32's accelerometer + gyro. Blue in standby, red in battle, flashing when
attacked; propellers spin faster as the drone gets busier; anti-aliased lines;
the twin barrels turn into laser beams when it fires. Left/right hold = turn the
aim, up = aim offset back to 0, centre again = radar.

**Zombie game (all in the FPGA).**
The IR sensor only SPAWNS zombies: when it sees something, a zombie appears far out
at a random angle and walks in (another every 4 s while the IR stays on, up to 4).
1 in 4 is a big one: slower, 3 hits. Killing depends only on AIM: turn the drone
(yaw) and fire the laser as usual; the nearest zombie within +-17 degrees of the
aim is hit. A zombie that reaches the drone bites (red bezel, growl, LEDs).
Radar: green dot / purple big dot, drone icon in the centre whose barrel shows
the aim. 3D: green column / thicker purple column.

The FPGA reports what happened back to the RP2350 on MISO (reply 0x5A + counters:
hits, kills, misses, near, small bites, big bites). code.py turns each change into
one letter for the drone (K kill, H big-zombie hit, M miss, A approaching,
B small bite = -1 HP, G big bite = -2 HP). The drone has 5 HP; at 0 HP it enters
the same ATTACKED / rescue loop as a flip.

## Files
| File | What it is |
|---|---|
| `../greymecha/cc_radar.bit` | Ready-built FPGA design (already on the GreyMecha CIRCUITPY drive). |
| `src/top.v` | Clock, packet receiver, timers, LED and buzzer effects, display hookup |
| `src/spi_status_rx.v` | Receives and checks the 32-byte status packet |
| `src/radar_render.v` | 6-stage pipeline that computes each pixel's colour |
| `src/wire3d.v` | 3D engine: per-frame vertex pipeline (rotate, view, perspective) + 32 parallel line testers + 4 zombie column testers |
| `src/zombies.v` | Zombie game: spawn, walk, aim check, hits, and where each zombie is drawn |
| `../greymecha/cc_radar_v1.bit` | The first (radar-only) design, kept as a fallback |
| `src/*.mem` | Lookup tables: atan2 angles, 5x7 font, text position maps |
| `src/oled_*.v`, `simple_spi_master.v`, `gc9a01_init_rom.vh`, `ecp5_oled_pll.v` | GC9A01 display driver, reused from Hackin7/GreyMechaArmy_Tests |
| `pinout.lpf` | FPGA pins used |
| `tools/gen_tables.py` | Regenerates the `.mem` tables |
| `tools/gen_wire_tables.py` | Regenerates `sin_rom.mem` and `rec_rom.mem` for the 3D view |
| `preview/*.png` | Screens rendered from simulation of the design |
| `sim/tb_*.v` | Testbenches (Icarus Verilog) used to verify each block |

## Status packet (RP2350 -> FPGA, SPI mode 0, 1 MHz)
| Byte | Meaning |
|---|---|
| 0 | 0xA5 sync |
| 1 | state: 0 standby, 1 battle, 2 attacked, 3 lost |
| 2 | laser charge, percent |
| 3 | bit0 IR target, bit1 toggles once per laser shot |
| 4 | compass heading, 0..255 = 0..360 degrees |
| 5 | rescue seconds left |
| 6 | hits |
| 7-18 | text line 1 (12 characters) |
| 19-30 | text line 2 (12 characters) |
| 31 | XOR of bytes 1..30 |

Attitude packet (5 bytes, sent by code.py for every `$A` line from the drone):
`0xA6, roll, pitch, yaw, XOR of bytes 1..3` (angles 0..255 = 0..360 degrees).
The v1 design ignores it (wrong length), so the new code.py also works with `cc_radar_v1.bit`.

## How the 3D view works (no frame buffer)
1. Once per frame (~140 clocks, during the last row): sin/cos table look-ups; the 32
   corners of the drone go through a pipeline - roll, pitch, yaw rotations, a camera
   30 degrees above, perspective (one corner per clock, 14 multipliers); the 8 zombie
   points (feet, heads) skip the rotations. Then each of the 32 lines gets its start
   values, and each zombie its column box.
2. Every pixel: 32 line testers and 4 column testers work in parallel. Each keeps two running sums
   (distance across the line, distance along it); one pixel to the right adds a
   constant, a new row adds another - so a tester is only adders and comparators.
   This is the "edge function" trick GPUs use. Nearer lines are drawn brighter.
   Within half a pixel of the line = full colour, up to one pixel = half colour
   (anti-aliasing).

Bad packets (wrong sync, wrong checksum, wrong length) are ignored. With no
good packet for 1.5 s the screen shows "NO LINK" over TV static.

## Rebuild (Windows, macOS or Linux)
```
pip install yowasp-yosys yowasp-nextpnr-ecp5
cd src
yowasp-yosys -p "synth_ecp5 -top top -json top.json" top.v spi_status_rx.v radar_render.v wire3d.v zombies.v oled_gc9a01.v oled_init.v oled_stream.v simple_spi_master.v ecp5_oled_pll.v
yowasp-nextpnr-ecp5 --25k --package CABGA256 --speed 6 --lpf ../pinout.lpf --json top.json --textcfg top.config
yowasp-ecppack --compress top.config ../cc_radar.bit
```
Then copy the new `cc_radar.bit` onto the CIRCUITPY drive. Check the nextpnr output says `PASS` for the clock (needs 77.5 MHz; this
design reaches 80-90 MHz; shipped build: 80 MHz, 48% of the logic, 25/28 multipliers).

## Verification done
- Renderer simulated for every screen state -> `preview/screens.png`
- Full system simulated: SPI packet in -> display powers up -> one whole
  frame captured from the screen pins -> matches the preview; 24.6 ms per frame
- Packet receiver rejects bad checksum, short packet and wrong sync byte
- Packets produced by `code.py` decoded correctly by the receiver
- Synthesis + place and route for ECP5-25K CABGA256: timing passes on 3 seeds

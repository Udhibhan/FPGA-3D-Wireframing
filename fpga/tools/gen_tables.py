"""Generate the FPGA lookup tables.
angle_rom.mem : 128x128 quadrant of atan2 -> 0..64 (64 = 90 degrees), addr = ay*128 + ax
font_rom.mem  : 5x7 font (same table as the STM32 OLED code), addr = (char-0x20)*8 + column
"""
import math, re, sys
src_c, out_dir = sys.argv[1], sys.argv[2]

with open(f"{out_dir}/angle_rom.mem", "w") as f:
    for ay in range(128):
        for ax in range(128):
            q = 0 if (ax == 0 and ay == 0) else round(math.atan2(ax, ay) / (math.pi / 2) * 64)
            f.write(f"{min(q, 64):02x}\n")

text = open(src_c, encoding="utf-8", errors="replace").read()
block = text[text.index("font5x7[64][5]"):]
rows = re.findall(r"\{(0x[0-9A-Fa-f]{2}(?:,\s*0x[0-9A-Fa-f]{2}){4})\}", block)[:64]
assert len(rows) == 64, len(rows)
with open(f"{out_dir}/font_rom.mem", "w") as f:
    for r in rows:
        cols = [int(v, 16) for v in r.split(",")]
        for c in cols + [0, 0, 0]:
            f.write(f"{c:02x}\n")
print("tables written")

# ---- position maps: replace division with lookup (one entry per x or y) ----
def w(name, vals):
    with open(f"{out_dir}/{name}", "w") as f:
        for v in vals:
            f.write(f"{v:02x}\n")

# text lines: 12 chars x 12 px starting at x=48; bit7 inside, bits6:3 char, bits2:0 column
xt = []
for x in range(256):
    n = x - 48
    xt.append((0x80 | ((n // 12) << 3) | ((n % 12) // 2)) if 0 <= n < 144 else 0)
w("map_text_x.mem", xt)
# big digits: bit7 inside, bit6 second char, bits2:0 column; one digit at x=100, two at x=76
def bigx(x0, nchars):
    out = []
    for x in range(256):
        n = x - x0
        out.append((0x80 | ((n // 48) << 6) | ((n % 48) // 8)) if 0 <= n < 48 * nchars - 8 else 0)
    return out
w("map_big1_x.mem", bigx(100, 1))
w("map_big2_x.mem", bigx(76, 2))
# y: bit7 line 1 (y 56..69), bit6 line 2 (y 170..183), bit5 big digits (y 92..147),
#    bits2:0 text row, bits4:3 unused; big-digit row in a separate map
yt, yb = [], []
for y in range(256):
    v = 0
    if 56 <= y < 70:   v |= 0x80 | ((y - 56) // 2)
    if 170 <= y < 184: v |= 0x40 | ((y - 170) // 2)
    if 92 <= y < 148:  v |= 0x20
    yt.append(v)
    yb.append(((y - 92) // 8) if 92 <= y < 148 else 0)
w("map_text_y.mem", yt)
w("map_big_y.mem", yb)
print("maps written")

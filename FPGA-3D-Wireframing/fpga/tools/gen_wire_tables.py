"""Lookup tables for the 3D wireframe (wire3d.v).

sin_rom.mem : 256 entries, sin(2*pi*i/256) * 256, 10-bit two's complement (cos = sin(i + 64))
rec_rom.mem : 256 entries indexed by depth w (-128..127 as a byte),
              perspective scale round(256 * D / (D + w)), 9 bits unsigned
"""
import math
import os

D = 360                      # camera distance in model units
out = os.path.join(os.path.dirname(__file__), "..", "src")

with open(os.path.join(out, "sin_rom.mem"), "w") as f:
    for i in range(256):
        v = int(round(256 * math.sin(2 * math.pi * i / 256)))
        f.write("%03x\n" % (v & 0x3FF))

with open(os.path.join(out, "rec_rom.mem"), "w") as f:
    for i in range(256):
        w = i - 256 if i >= 128 else i
        f.write("%03x\n" % int(round(256 * D / (D + w))))

print("wrote sin_rom.mem and rec_rom.mem")

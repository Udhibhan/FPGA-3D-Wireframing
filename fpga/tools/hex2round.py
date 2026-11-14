"""RGB565 240x240 hex dump -> 480x480 PNG with the round panel's transparent corners."""
import sys
from PIL import Image, ImageDraw
src, dst = sys.argv[1], sys.argv[2]
vals = [int(l, 16) for l in open(src) if l.strip()]
im = Image.new("RGB", (240, 240)); px = im.load()
for k, v in enumerate(vals):
    px[k % 240, k // 240] = (((v >> 11) & 31) * 255 // 31, ((v >> 5) & 63) * 255 // 63, (v & 31) * 255 // 31)
im = im.resize((480, 480), Image.NEAREST).convert("RGBA")
mask = Image.new("L", (480, 480), 0); ImageDraw.Draw(mask).ellipse((0, 0, 479, 479), fill=255)
im.putalpha(mask); im.save(dst); print("saved", dst)

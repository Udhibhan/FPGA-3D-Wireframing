"""Convert RGB565 hex dumps (one pixel per line, 240x240) to PNG, plus a contact sheet."""
import sys
from PIL import Image, ImageDraw
def load(path):
    img = Image.new("RGB", (240, 240))
    px = img.load()
    vals = [int(l, 16) for l in open(path) if l.strip()]
    for k, v in enumerate(vals):
        r = (v >> 11) & 31; g = (v >> 5) & 63; b = v & 31
        px[k % 240, k // 240] = (r * 255 // 31, g * 255 // 63, b * 255 // 31)
    return img
out, items = sys.argv[1], sys.argv[2:]
pairs = [items[i:i+2] for i in range(0, len(items), 2)]
tiles = []
for path, label in pairs:
    im = load(path)
    # round display: mask the corners like the real panel
    mask = Image.new("L", (240, 240), 0); ImageDraw.Draw(mask).ellipse((0, 0, 239, 239), fill=255)
    tile = Image.new("RGB", (260, 290), (24, 24, 28)); tile.paste(im, (10, 10), mask)
    ImageDraw.Draw(tile).text((10, 260), label, fill=(220, 220, 220))
    tiles.append(tile)
cols = 4; rows = (len(tiles) + cols - 1) // cols
sheet = Image.new("RGB", (260 * cols, 290 * rows), (24, 24, 28))
for i, t in enumerate(tiles):
    sheet.paste(t, ((i % cols) * 260, (i // cols) * 290))
sheet = sheet.resize((sheet.width * 2, sheet.height * 2), Image.NEAREST)
sheet.save(out); print("saved", out, sheet.size)

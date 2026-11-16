# Tutorial 7's rut figure (docs/media/t7_ruts.webp) from a saved rut set, state/<map>/ruts/runs/*.npz
# (swb_isaac_world soil.save_ruts: "<zone>:<bx>:<by>" 16x16 float32 plastic sinkage in m).
#   uv run --no-project --with matplotlib --with numpy python docs/figures/t7_ruts.py <set.npz> ruts.png
#   ffmpeg -i ruts.png -lossless 1 docs/media/t7_ruts.webp
import json, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
src, out = sys.argv[1], sys.argv[2]
z = np.load(src)
meta = json.loads(str(z["__meta__"]))
B, h = meta["block"], meta["spacing"]["mud_road"]
x0, x1, y0, y1 = 144.0, 160.0, -135.0, -127.0
nx, ny = round((x1 - x0) / h), round((y1 - y0) / h)
grid = np.full((ny, nx), np.nan)
for k in z.files:
    if k == "__meta__": continue
    _, bx, by = k.split(":"); bx, by = int(bx), int(by)
    a = z[k]
    for i in range(B):
        for j in range(B):
            x, y = (B*bx+i)*h, (B*by+j)*h
            c, r = round((x-x0)/h), round((y-y0)/h)
            if 0 <= c < nx and 0 <= r < ny: grid[r, c] = a[i, j]*100
print("max cm", np.nanmax(grid), "blocks", len(z.files)-1)
grid = np.where(np.isnan(grid), 0.0, grid)
X = x0 + (np.arange(nx)+0.5)*h; Y = y0 + (np.arange(ny)+0.5)*h
inside = ((X[None,:] >= 145.6) & (X[None,:] <= 158.4) & (Y[:,None] >= -134.4) & (Y[:,None] <= -128.0))
grid = np.ma.masked_where(~inside, grid)
cm = plt.get_cmap("magma_r").copy(); cm.set_bad("#e6e6e6")
fig, ax = plt.subplots(figsize=(9, 4.4), dpi=130)
im = ax.imshow(grid, origin="lower", extent=(x0, x1, y0, y1), cmap=cm, vmin=0, vmax=1.5, interpolation="nearest")
ax.add_patch(Rectangle((145.6, -134.4), 12.8, 6.4, fill=False, ec="#2b5fb0", lw=1.2, ls="--"))
ax.text(145.8, -134.2, "mud_road zone (outside: no soil)", color="#2b5fb0", fontsize=9, va="bottom")
ax.annotate("", xy=(150.5, -133.0), xytext=(154.5, -133.0), arrowprops=dict(arrowstyle="->", color="#444"))
ax.text(152.5, -132.8, "driving direction (west)", ha="center", fontsize=9, color="#444")
ax.text(158.2, -129.9, "left wheels", ha="right", va="bottom", fontsize=9)
ax.text(158.2, -128.3, "right wheels", ha="right", va="bottom", fontsize=9)
ax.set_xlabel("map x (m)"); ax.set_ylabel("map y (m)"); ax.set_aspect("equal")
cb = fig.colorbar(im, ax=ax, shrink=0.8, pad=0.02); cb.set_label("rut depth (cm)")
fig.tight_layout(); fig.savefig(out)

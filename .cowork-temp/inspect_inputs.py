import rasterio, os
from pathlib import Path
import numpy as np

p = Path(r'D:\WorkFile\3DMapRebuilder\TestInput\SanHe')
for name in ['dem.tif','labels.tif','building.tif','road.tif','water.tif','satellite.tif']:
    f = p/name
    if f.exists():
        with rasterio.open(f) as src:
            w,h = src.width, src.height
            bb = src.bounds
            res = src.res
            area = (bb.right-bb.left)*(bb.top-bb.bottom)/1e6
            print(f"{name}: {w}x{h} px, res={res}, bounds=({bb.left:.1f},{bb.bottom:.1f},{bb.right:.1f},{bb.top:.1f}), area={area:.4f} km^2")

print('---OBJ sizes---')
for name in ['terrain_ground.obj','terrain_building.obj','terrain_water.obj']:
    f = Path(r'D:\WorkFile\3DMapRebuilder\output\terrain_adaptive')/name
    if f.exists():
        print(f"{name}: {f.stat().st_size/1e6:.2f} MB")

print('---label class counts---')
with rasterio.open(p/'labels.tif') as src:
    labels = src.read(1)
classes, counts = np.unique(labels, return_counts=True)
for c,n in zip(classes, counts):
    print(f"class {c}: {n} px ({n/labels.size*100:.2f}%)")

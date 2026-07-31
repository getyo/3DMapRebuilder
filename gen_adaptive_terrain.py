#!/usr/bin/env python3
"""
gen_adaptive_terrain.py -- 自适应分辨率 UE 地形生成器

策略：
  - 用 LabelPostprocessor 生成/确认 labels_10x_no_boundary.tif 和 boundary_lines_10x.png
  - 从 boundary_lines_10x.png 提取 1 像素宽的 10x 边界线
  - 粗网格顶点 + 密集边界点 → scipy.spatial.Delaunay 三角化
  - 三角形大小自然自适应：内部大，边界小
  - 每个三角形按重心 class_id 分到 4 个材质槽之一
  - 不做高斯平滑
  - 水体保留凹包/盆地
  - 坐标系与 gen_splat.py 一致：X=east, Y=north, Z=height
"""

import os
import sys
import time
import argparse

import numpy as np
import rasterio
import cv2
from scipy.ndimage import distance_transform_edt
from scipy.spatial import Delaunay

# ═══════════════════════════════════════════════════════════════
# 配置
# ═══════════════════════════════════════════════════════════════

COARSE_STRIDE_ORIG = 10          # 粗网格在原始像素下的 stride
SCALE = 10                       # 10x 上采样倍数
BOUNDARY_SAMPLE_STEP = 1         # 沿边界线采样步长（10x 像素）

WATER_DEPTH = 7.5                # 水体最大深度（米）
WATER_EDGE_WIDTH = 12.0          # 水边过渡宽度（米）
PX_M = 0.458                     # 原始像素地面尺寸（米）
UE_SCALE = 100.0                 # UE 单位缩放

MAT_NAMES = ["M_Ground", "M_Road", "M_Building", "M_Water"]
MAT_COLORS = [
    (0.55, 0.45, 0.30),   # Ground
    (0.28, 0.28, 0.30),   # Road
    (0.82, 0.78, 0.70),   # Building
    (0.10, 0.18, 0.28),   # Water
]
MAT_GROUND = 0
MAT_ROAD = 1
MAT_BUILDING = 2
MAT_WATER = 3


def class_to_mat(class_id):
    if class_id == 0:
        return MAT_WATER
    if class_id == 20:
        return MAT_ROAD
    if class_id == 40:
        return MAT_BUILDING
    return MAT_GROUND


# ═══════════════════════════════════════════════════════════════
# 自适应地形构建器
# ═══════════════════════════════════════════════════════════════

class AdaptiveTerrainBuilder:
    def __init__(
        self,
        label_10x_path: str,
        boundary_path: str,
        dem_path: str,
        out_dir: str,
        coarse_stride_orig: int = COARSE_STRIDE_ORIG,
        boundary_sample_step: int = BOUNDARY_SAMPLE_STEP,
        water_depth: float = WATER_DEPTH,
        water_edge_width: float = WATER_EDGE_WIDTH,
        ue_scale: float = UE_SCALE,
    ):
        self.label_10x_path = label_10x_path
        self.boundary_path = boundary_path
        self.dem_path = dem_path
        self.out_dir = out_dir
        self.CS_orig = coarse_stride_orig
        self.CS_10x = coarse_stride_orig * SCALE
        self.bd_step = boundary_sample_step
        self.water_depth = water_depth
        self.water_edge_width = water_edge_width
        self.ue_scale = ue_scale

        self.H10 = self.W10 = 0
        self.class_10x = None
        self.dem_10x = None
        self.water_mask = None
        self.water_depth_map = None
        self.boundary_mask = None

        self.pts = None        # Delaunay 输入点 (r, c)
        self.tri = None        # Delaunay 对象
        self.verts = []        # (x, y, z, u, v)
        self.faces = []        # (i0, i1, i2, mat_id)

    # ═══════════════════════════════════════════════════════════════
    # 加载与预处理
    # ═══════════════════════════════════════════════════════════════

    def load(self):
        print(f"Loading 10x labels: {self.label_10x_path}")
        with rasterio.open(self.label_10x_path) as src:
            self.class_10x = src.read(1).astype(np.uint8)
        self.H10, self.W10 = self.class_10x.shape
        print(f"  size: {self.W10}x{self.H10}")

        print(f"Loading DEM: {self.dem_path}")
        with rasterio.open(self.dem_path) as src:
            dem_1x = src.read(1).astype(np.float32)
        self.dem_10x = cv2.resize(dem_1x, (self.W10, self.H10), interpolation=cv2.INTER_LINEAR)

        self.water_mask = (self.class_10x == 0)
        self._build_water_basin()
        self._build_boundary_mask()

    def _build_water_basin(self):
        dist_px = distance_transform_edt(self.water_mask)
        dist_m = dist_px * (PX_M / SCALE)
        t = np.clip(dist_m / self.water_edge_width, 0.0, 1.0)
        self.water_depth_map = self.water_depth * (t * t * (3.0 - 2.0 * t))
        max_depth = self.water_depth_map[self.water_mask].max()
        print(f"  Water basin depth: 0.0 ~ {max_depth:.2f} m")

    def _build_boundary_mask(self):
        print(f"Loading boundary: {self.boundary_path}")
        img = cv2.imread(self.boundary_path, cv2.IMREAD_UNCHANGED)
        if len(img.shape) == 3 and img.shape[2] == 4:
            mask = (img[:, :, 3] > 0).astype(np.uint8)
        else:
            mask = (img > 0).astype(np.uint8)

        print(f"  raw boundary pixels: {mask.sum():,}")
        thinned = cv2.ximgproc.thinning(mask * 255)
        self.boundary_mask = (thinned > 0)
        print(f"  thinned boundary pixels: {self.boundary_mask.sum():,}")

    # ═══════════════════════════════════════════════════════════════
    # 点集与 Delaunay
    # ═══════════════════════════════════════════════════════════════

    def build_delaunay(self):
        CS = self.CS_10x

        # 粗网格顶点
        grid_r = np.arange(0, self.H10, CS, dtype=np.int64)
        grid_c = np.arange(0, self.W10, CS, dtype=np.int64)
        if grid_r[-1] != self.H10 - 1:
            grid_r = np.append(grid_r, self.H10 - 1)
        if grid_c[-1] != self.W10 - 1:
            grid_c = np.append(grid_c, self.W10 - 1)

        gr, gc = np.meshgrid(grid_r, grid_c, indexing='ij')
        grid_pts = np.column_stack([gr.ravel(), gc.ravel()])

        # 边界点（采样）
        br, bc = np.where(self.boundary_mask)
        if self.bd_step > 1:
            idx = np.arange(0, len(br), self.bd_step)
            br, bc = br[idx], bc[idx]
        bd_pts = np.column_stack([br, bc])

        # 合并并去重
        pts = np.unique(np.vstack([grid_pts, bd_pts]), axis=0)
        self.pts = pts.astype(np.float64)
        print(f"  Total Delaunay points: {len(self.pts):,}  (grid {len(grid_pts):,}, boundary {len(bd_pts):,})")

        print("Running Delaunay...")
        t0 = time.time()
        self.tri = Delaunay(self.pts)
        print(f"  Delaunay: {len(self.tri.simplices):,} triangles in {time.time()-t0:.1f}s")

    # ═══════════════════════════════════════════════════════════════
    # 网格生成
    # ═══════════════════════════════════════════════════════════════

    def build_mesh(self):
        print("Building mesh...")
        t0 = time.time()

        H, W = self.H10, self.W10

        # 预计算所有点的 3D 顶点
        for r, c in self.pts:
            r = int(round(r))
            c = int(round(c))
            r = np.clip(r, 0, H - 1)
            c = np.clip(c, 0, W - 1)

            # UE 左手系：X=east, Y=-north, Z=height
            x = (c / SCALE) * PX_M * self.ue_scale
            y = -((r / SCALE) * PX_M * self.ue_scale)
            x -= (W / SCALE) * PX_M * self.ue_scale * 0.5
            y += (H / SCALE) * PX_M * self.ue_scale * 0.5

            cls = self.class_10x[r, c]
            dem_h = self.dem_10x[r, c]
            if cls == 0:
                dem_h -= self.water_depth_map[r, c]
            z = dem_h * self.ue_scale

            u = c / (W - 1)
            v = 1.0 - r / (H - 1)

            self.verts.append((x, y, z, u, v))

        # 遍历三角形
        for tri in self.tri.simplices:
            i0, i1, i2 = tri
            p0 = self.pts[i0]
            p1 = self.pts[i1]
            p2 = self.pts[i2]

            # 重心
            cr = (p0[0] + p1[0] + p2[0]) / 3.0
            cc = (p0[1] + p1[1] + p2[1]) / 3.0
            cr_i = int(round(np.clip(cr, 0, H - 1)))
            cc_i = int(round(np.clip(cc, 0, W - 1)))

            mat = class_to_mat(self.class_10x[cr_i, cc_i])

            # 保证朝向一致：2D 叉积 > 0
            cross = (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p1[1] - p0[1]) * (p2[0] - p0[0])
            if cross < 0:
                i0, i1 = i1, i0

            self.faces.append((i0, i1, i2, mat))

        print(f"  Mesh: {len(self.verts):,} vertices, {len(self.faces):,} triangles in {time.time()-t0:.1f}s")

    # ═══════════════════════════════════════════════════════════════
    # 法线与导出
    # ═══════════════════════════════════════════════════════════════

    def _compute_normals(self):
        verts = np.array([[v[0], v[1], v[2]] for v in self.verts], dtype=np.float64)
        norms = np.zeros((len(self.verts), 3), dtype=np.float64)

        for i0, i1, i2, _ in self.faces:
            p0 = verts[i0]
            p1 = verts[i1]
            p2 = verts[i2]
            fn = np.cross(p1 - p0, p2 - p0)
            nm = np.linalg.norm(fn)
            if nm > 1e-12:
                fn /= nm
            norms[i0] += fn
            norms[i1] += fn
            norms[i2] += fn

        mag = np.linalg.norm(norms, axis=1, keepdims=True)
        mag[mag < 1e-12] = 1.0
        norms /= mag
        return norms

    def export(self):
        os.makedirs(self.out_dir, exist_ok=True)
        obj_path = os.path.join(self.out_dir, "terrain_adaptive.obj")
        mtl_path = os.path.join(self.out_dir, "terrain_adaptive.mtl")

        print("Computing normals...")
        norms = self._compute_normals()

        print(f"Writing MTL: {mtl_path}")
        with open(mtl_path, "w", encoding="utf-8") as mf:
            for name, col in zip(MAT_NAMES, MAT_COLORS):
                mf.write(f"newmtl {name}\n")
                mf.write(f"Kd {col[0]:.4f} {col[1]:.4f} {col[2]:.4f}\n")
                mf.write("Ks 0.1 0.1 0.1\n")
                mf.write("Ns 32.0\n\n")

        print(f"Writing OBJ: {obj_path}")
        with open(obj_path, "w", encoding="utf-8") as of:
            of.write("# Adaptive terrain from 10x labels (Delaunay + dense boundary)\n")
            of.write(f"mtllib {os.path.basename(mtl_path)}\n")
            of.write("o Terrain\n")

            for x, y, z, _, _ in self.verts:
                of.write(f"v {x:.6f} {y:.6f} {z:.6f}\n")
            for n in norms:
                of.write(f"vn {n[0]:.6f} {n[1]:.6f} {n[2]:.6f}\n")
            for _, _, _, u, v in self.verts:
                of.write(f"vt {u:.6f} {v:.6f}\n")

            faces_by_mat = [[] for _ in MAT_NAMES]
            for f in self.faces:
                faces_by_mat[f[3]].append(f)

            for mat_id, name in enumerate(MAT_NAMES):
                if not faces_by_mat[mat_id]:
                    continue
                of.write(f"usemtl {name}\n")
                for i0, i1, i2, _ in faces_by_mat[mat_id]:
                    of.write(f"f {i0+1}/{i0+1}/{i0+1} {i1+1}/{i1+1}/{i1+1} {i2+1}/{i2+1}/{i2+1}\n")

        print(f"Done! OBJ: {obj_path} ({os.path.getsize(obj_path)/1e6:.1f} MB)")

    def build(self):
        t0 = time.time()
        self.load()
        self.build_delaunay()
        self.build_mesh()
        self.export()
        print(f"Total time: {time.time()-t0:.1f}s")


# ═══════════════════════════════════════════════════════════════
# 入口
# ═══════════════════════════════════════════════════════════════

def main():
    p = argparse.ArgumentParser(description="Adaptive UE terrain generator")
    p.add_argument("--label-dir", default="TestInput/SanHe", help="labels.tif 所在目录")
    p.add_argument("--dem", default="TestInput/SanHe/dem.tif", help="DEM 路径")
    p.add_argument("--out-dir", default="output/terrain_adaptive", help="输出目录")
    p.add_argument("--boundary-step", type=int, default=BOUNDARY_SAMPLE_STEP, help="边界采样步长")
    args = p.parse_args()

    label_10x_path = os.path.join(args.label_dir, "Output_10x", "labels_10x_no_boundary.tif")
    boundary_path = os.path.join(args.label_dir, "Output_10x", "boundary_lines_10x.png")

    builder = AdaptiveTerrainBuilder(
        label_10x_path=label_10x_path,
        boundary_path=boundary_path,
        dem_path=args.dem,
        out_dir=args.out_dir,
        boundary_sample_step=args.boundary_step,
    )
    builder.build()


if __name__ == "__main__":
    main()

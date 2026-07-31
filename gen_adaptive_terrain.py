#!/usr/bin/env python3
"""
gen_adaptive_terrain.py -- 自适应分辨率 UE 地形生成器

本文件是完整管线的唯一入口（main），负责：
  1. 语义分类（classify_vecw.py）
  2. 标签后处理（label_postprocess.py）
  3. 简化 3DGS 语义地图（gen_semantic.py）
  4. 自适应分辨率地形 OBJ 生成（本文件 AdaptiveTerrainBuilder）

其他模块仅通过 test() 方法自检输入并运行，不持有 main 入口。
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

# 添加脚本目录到导入路径，确保 standalone 运行能找到同级模块
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from classify_vecw import VecClassifier
from gen_semantic import SemanticMapBuilder
from label_postprocess import LabelPostprocessor


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
    (0.55, 0.45, 0.30),   # 地面
    (0.28, 0.28, 0.30),   # 道路
    (0.82, 0.78, 0.70),   # 建筑
    (0.10, 0.18, 0.28),   # 水体
]
MAT_GROUND = 0
MAT_ROAD = 1
MAT_BUILDING = 2
MAT_WATER = 3


def class_to_mat(class_id):
    """class_id → 材质槽编号"""
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
    """
    从 10x 语义标签、边界线图和 DEM 生成 UE 可用的自适应分辨率地形网格。
    内部采用 Delaunay + 密集边界点策略：粗网格顶点保证内部低面数，
    边界点密集采样保证边界平滑。
    """

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

        self.pts = None
        self.tri = None
        self.verts = []
        self.faces = []

    # ═══════════════════════════════════════════════════════════════
    # 加载与预处理
    # ═══════════════════════════════════════════════════════════════

    def load(self):
        """加载 10x 标签、DEM 与边界线，并构建水体盆地。"""
        print(f"加载 10x 标签: {self.label_10x_path}")
        with rasterio.open(self.label_10x_path) as src:
            self.class_10x = src.read(1).astype(np.uint8)
        self.H10, self.W10 = self.class_10x.shape
        print(f"  尺寸: {self.W10}x{self.H10}")

        print(f"加载 DEM: {self.dem_path}")
        with rasterio.open(self.dem_path) as src:
            dem_1x = src.read(1).astype(np.float32)
        self.dem_10x = cv2.resize(dem_1x, (self.W10, self.H10), interpolation=cv2.INTER_LINEAR)

        self.water_mask = (self.class_10x == 0)
        self._build_water_basin()
        self._build_boundary_mask()

    def _build_water_basin(self):
        """基于距离变换生成水体凹包/盆地深度图。"""
        dist_px = distance_transform_edt(self.water_mask)
        dist_m = dist_px * (PX_M / SCALE)
        t = np.clip(dist_m / self.water_edge_width, 0.0, 1.0)
        self.water_depth_map = self.water_depth * (t * t * (3.0 - 2.0 * t))
        max_depth = self.water_depth_map[self.water_mask].max()
        print(f"  水体盆地深度: 0.0 ~ {max_depth:.2f} m")

    def _build_boundary_mask(self):
        """从 boundary_lines_10x.png 提取 1 像素宽的边界线。"""
        print(f"加载边界线: {self.boundary_path}")
        img = cv2.imread(self.boundary_path, cv2.IMREAD_UNCHANGED)
        if img is None:
            raise FileNotFoundError(f"边界线图不存在: {self.boundary_path}")
        if len(img.shape) == 3 and img.shape[2] == 4:
            mask = (img[:, :, 3] > 0).astype(np.uint8)
        else:
            mask = (img > 0).astype(np.uint8)

        print(f"  原始边界像素: {mask.sum():,}")
        # 不依赖 cv2.ximgproc，用轮廓重绘实现 1 像素细化
        contours, _ = cv2.findContours(mask * 255, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        thinned = np.zeros_like(mask, dtype=np.uint8)
        cv2.drawContours(thinned, contours, -1, 255, thickness=1)
        self.boundary_mask = (thinned > 0)
        print(f"  细化边界像素: {self.boundary_mask.sum():,}")

    # ═══════════════════════════════════════════════════════════════
    # 点集与 Delaunay
    # ═══════════════════════════════════════════════════════════════

    def build_delaunay(self):
        """构建粗网格顶点 + 密集边界点，执行 Delaunay 三角化。"""
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

        # 边界点采样
        br, bc = np.where(self.boundary_mask)
        if self.bd_step > 1:
            idx = np.arange(0, len(br), self.bd_step)
            br, bc = br[idx], bc[idx]
        bd_pts = np.column_stack([br, bc])

        # 合并去重
        pts = np.unique(np.vstack([grid_pts, bd_pts]), axis=0)
        self.pts = pts.astype(np.float64)
        print(f"  Delaunay 总点数: {len(self.pts):,} (粗网格 {len(grid_pts):,}, 边界 {len(bd_pts):,})")

        print("执行 Delaunay 三角化...")
        t0 = time.time()
        self.tri = Delaunay(self.pts)
        print(f"  三角形数: {len(self.tri.simplices):,}, 耗时 {time.time()-t0:.1f}s")

    # ═══════════════════════════════════════════════════════════════
    # 网格生成
    # ═══════════════════════════════════════════════════════════════

    def build_mesh(self):
        """根据 Delaunay 结果生成带材质的三维网格。"""
        print("构建三维网格...")
        t0 = time.time()
        H, W = self.H10, self.W10

        # 预计算顶点
        for r, c in self.pts:
            r = int(round(r))
            c = int(round(c))
            r = np.clip(r, 0, H - 1)
            c = np.clip(c, 0, W - 1)

            # UE 左手系 Z-up：X=east, Y=-north, Z=height
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

        # 遍历三角形，按重心 class 分配材质
        for tri in self.tri.simplices:
            i0, i1, i2 = tri
            p0 = self.pts[i0]
            p1 = self.pts[i1]
            p2 = self.pts[i2]

            cr = (p0[0] + p1[0] + p2[0]) / 3.0
            cc = (p0[1] + p1[1] + p2[1]) / 3.0
            cr_i = int(round(np.clip(cr, 0, H - 1)))
            cc_i = int(round(np.clip(cc, 0, W - 1)))
            mat = class_to_mat(self.class_10x[cr_i, cc_i])

            # 统一朝向
            cross = (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p1[1] - p0[1]) * (p2[0] - p0[0])
            if cross < 0:
                i0, i1 = i1, i0

            self.faces.append((i0, i1, i2, mat))

        print(f"  顶点: {len(self.verts):,}, 三角形: {len(self.faces):,}, 耗时 {time.time()-t0:.1f}s")

    # ═══════════════════════════════════════════════════════════════
    # 法线与导出
    # ═══════════════════════════════════════════════════════════════

    def _compute_normals(self):
        """按面法线加权平均计算顶点法线。"""
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
        """导出 OBJ + MTL。"""
        os.makedirs(self.out_dir, exist_ok=True)
        obj_path = os.path.join(self.out_dir, "terrain_adaptive.obj")
        mtl_path = os.path.join(self.out_dir, "terrain_adaptive.mtl")

        print("计算法线...")
        norms = self._compute_normals()

        print(f"写入 MTL: {mtl_path}")
        with open(mtl_path, "w", encoding="utf-8") as mf:
            for name, col in zip(MAT_NAMES, MAT_COLORS):
                mf.write(f"newmtl {name}\n")
                mf.write(f"Kd {col[0]:.4f} {col[1]:.4f} {col[2]:.4f}\n")
                mf.write("Ks 0.1 0.1 0.1\n")
                mf.write("Ns 32.0\n\n")

        print(f"写入 OBJ: {obj_path}")
        with open(obj_path, "w", encoding="utf-8") as of:
            of.write("# 自适应地形（Delaunay + 密集边界点）\n")
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

        print(f"完成! OBJ: {obj_path} ({os.path.getsize(obj_path)/1e6:.1f} MB)")

    def build(self):
        """完整构建流程。"""
        t0 = time.time()
        self.load()
        self.build_delaunay()
        self.build_mesh()
        self.export()
        print(f"总耗时: {time.time()-t0:.1f}s")


# ═══════════════════════════════════════════════════════════════
# 完整管线入口（唯一 main）
# ═══════════════════════════════════════════════════════════════

def main():
    """完整流程入口：分类 → 后处理 → 3DGS语义地图 → 自适应地形OBJ。"""
    p = argparse.ArgumentParser(description="3DMapRebuilder 自适应地形生成器")
    p.add_argument("--label-dir", default="TestInput/SanHe", help="labels.tif 所在目录")
    p.add_argument("--dem", default="TestInput/SanHe/dem.tif", help="DEM 路径")
    p.add_argument("--out-dir", default="output/terrain_adaptive", help="地形 OBJ 输出目录")
    p.add_argument("--boundary-step", type=int, default=BOUNDARY_SAMPLE_STEP, help="边界采样步长")
    p.add_argument("--force", action="store_true",
                   help="强制重新生成所有中间文件；默认会利用已有文件")
    args = p.parse_args()

    label_dir = args.label_dir
    label_path = os.path.join(label_dir, "labels.tif")
    out_10x = os.path.join(label_dir, "Output_10x")
    label_10x_path = os.path.join(out_10x, "labels_10x_no_boundary.tif")
    boundary_path = os.path.join(out_10x, "boundary_lines_10x.png")
    semantic_ply = os.path.join(label_dir, "semanticMap.ply")

    vec_path = os.path.join(label_dir, "vec_raw.png")
    sate_path = os.path.join(label_dir, "satellite.tif")

    # 1. 语义分类
    if not args.force and os.path.exists(label_path):
        print(f"使用已有: {label_path}")
    else:
        print("=" * 60)
        print("Stage 1/4: 语义分类")
        print("=" * 60)
        clf = VecClassifier()
        clf.set_input(vec_path, sate_path)
        clf.set_output(label_dir)
        clf.run()

    # 2. 标签后处理
    if not args.force and os.path.exists(label_10x_path) and os.path.exists(boundary_path):
        print(f"使用已有: {label_10x_path}, {boundary_path}")
    else:
        print("=" * 60)
        print("Stage 2/4: 标签后处理")
        print("=" * 60)
        pp = LabelPostprocessor(input_path=label_path, output_dir=out_10x)
        pp.process()

    # 3. 简化 3DGS 语义地图
    if not args.force and os.path.exists(semantic_ply):
        print(f"使用已有: {semantic_ply}")
    else:
        print("=" * 60)
        print("Stage 3/4: 简化 3DGS 语义地图")
        print("=" * 60)
        builder = SemanticMapBuilder(label_path=label_path, dem_path=args.dem, out_path=semantic_ply)
        builder.build()

    # 4. 自适应地形 OBJ
    print("=" * 60)
    print("Stage 4/4: 自适应地形 OBJ")
    print("=" * 60)
    terrain_builder = AdaptiveTerrainBuilder(
        label_10x_path=label_10x_path,
        boundary_path=boundary_path,
        dem_path=args.dem,
        out_dir=args.out_dir,
        boundary_sample_step=args.boundary_step,
    )
    terrain_builder.build()


def test():
    """完整管线测试入口（默认重新生成所有文件）。"""
    main()


if __name__ == "__main__":
    main()

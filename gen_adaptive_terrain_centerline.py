#!/usr/bin/env python3
"""
gen_adaptive_terrain_centerline.py — 自适应分辨率 UE 地形生成器 + 全管线编排入口

职责划分：
  1. 全管线编排入口（main）：分类 → 后处理 → 三路并行
     （3DGS 语义地图 gen_semantic / 地形 OBJ 本文件 / 水体流场 gen_water_flow）
  2. 地形构建 AdaptiveTerrainBuilder：从 10x 标签 + 边界线 + DEM 生成
     terrain_ground.obj / terrain_water.obj / terrain_building.obj

水体中心线（CSV）与静态速度场纹理已独立为 gen_water_flow.py（WaterFlowBuilder），
与地形 OBJ 无数据依赖，二者在并行阶段同时执行。坐标常量统一见 map_common.py。

classify_vecw / label_postprocess / gen_semantic / gen_water_flow
均可通过各自的 main() 独立运行。
"""

import argparse
import os
import queue
import re
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rasterio
import cv2
from scipy.ndimage import distance_transform_edt
from scipy.spatial import Delaunay

# 添加脚本目录到导入路径，确保 standalone 运行能找到同级模块
sys.path.insert(0, str(Path(__file__).parent))
from classify_vecw import VecClassifier
from label_postprocess import LabelPostprocessor
from map_common import PX_M, SCALE, UE_SCALE


# ═══════════════════════════════════════════════════════════════
# 配置
# ═══════════════════════════════════════════════════════════════

COARSE_STRIDE_ORIG = 10          # 粗网格在原始像素下的 stride
BOUNDARY_SAMPLE_STEP = 1         # 沿边界线采样步长（10x 像素）

WATER_DEPTH = 7.5                # 水体最大深度（米）
WATER_EDGE_WIDTH = 12.0          # 水边过渡宽度（米）
BUILDING_BUFFER_DEPTH = 0.01     # 建筑裙边最大下沉深度（米）

MAT_NAMES = ["M_Ground", "M_Road", "M_WaterBed", "M_Building"]
MAT_COLORS = [
    (0.55, 0.45, 0.30),   # 地面
    (0.28, 0.28, 0.30),   # 道路
    (0.16, 0.13, 0.09),   # 水底泥沙
    (0.82, 0.78, 0.70),   # 建筑
]
MAT_GROUND = 0
MAT_ROAD = 1
MAT_WATER_BED = 2
MAT_BUILDING_GROUND = 3

WATER_MAT_NAMES = ["M_Water"]
WATER_MAT_COLORS = [
    (0.10, 0.18, 0.28),   # 水面蓝色
]
MAT_WATER = 0

BUILDING_MAT_NAMES = ["M_Building"]
BUILDING_MAT_COLORS = [
    (0.82, 0.78, 0.70),   # 建筑
]
MAT_BUILDING = 0


def class_to_mat(class_ids):
    """class_id → 地面 OBJ 材质槽编号（向量化）"""
    result = np.full(np.asarray(class_ids).shape, MAT_GROUND, dtype=np.int32)
    result[class_ids == 0] = MAT_WATER_BED
    result[class_ids == 20] = MAT_ROAD
    return result


# ═══════════════════════════════════════════════════════════════
# 自适应地形构建器（仅地形网格；水体流场见 gen_water_flow.py）
# ═══════════════════════════════════════════════════════════════

class AdaptiveTerrainBuilder:
    """
    从 10x 语义标签、边界线图和 DEM 生成 UE 可用的自适应分辨率地形网格。
    输出三个 OBJ：
      - terrain_ground.obj：地面/道路/水体凹包 + 建筑裙边
      - terrain_building.obj：建筑核心区域顶面
      - terrain_water.obj：蓝色水面盖子
    三个 OBJ 同坐标系，UE 000 对齐。

    坐标常量（PX_M / SCALE / UE_SCALE）来自 map_common.py，
    与 gen_water_flow.py 的中心线 CSV / 速度场保持一致。
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
        building_buffer_depth: float = BUILDING_BUFFER_DEPTH,
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
        self.building_buffer_depth = building_buffer_depth
        self.ue_scale = ue_scale

        self.H10 = self.W10 = 0
        self.class_10x = None
        self.dem_10x = None
        self.water_mask = None
        self.water_depth_map = None
        self.building_mask = None
        self.building_dist_m = None
        self.building_depth_map = None
        self.boundary_mask = None

        self.pts = None
        self.tri = None
        self.verts = []            # 地面 OBJ 顶点（共享顶点）
        self.ground_faces = []     # 地面 OBJ 面
        self.water_verts = []      # 水面 OBJ 顶点
        self.water_faces = []      # 水面 OBJ 面
        self.building_verts = []   # 建筑 OBJ 顶点
        self.building_faces = []   # 建筑 OBJ 面

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
        self.building_mask = (self.class_10x == 40)
        self._build_water_basin()
        self._build_building_buffer()
        self._build_boundary_mask()

    def _build_water_basin(self):
        """基于距离变换生成水体凹包/盆地深度图。"""
        dist_px = distance_transform_edt(self.water_mask)
        dist_m = dist_px * (PX_M / SCALE)
        t = np.clip(dist_m / self.water_edge_width, 0.0, 1.0)
        self.water_depth_map = self.water_depth * (t * t * (3.0 - 2.0 * t))
        max_depth = self.water_depth_map[self.water_mask].max()
        print(f"  水体盆地深度: 0.0 ~ {max_depth:.2f} m")

    def _build_building_buffer(self):
        """基于距离变换生成建筑边界距离图（后续根据裙边范围再算深度）。"""
        dist_px = distance_transform_edt(self.building_mask)
        self.building_dist_m = dist_px * (PX_M / SCALE)
        print(f"  建筑边界距离图已构建")

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
        """根据 Delaunay 结果生成地面网格、建筑网格和水面盖子网格。"""
        print("构建三维网格...")
        t0 = time.time()
        H, W = self.H10, self.W10

        # 第一步：收集每个三角形的重心 class（向量化）
        simplices = self.tri.simplices
        pts_tri = self.pts[simplices]
        centroids = pts_tri.mean(axis=1)
        cr_i = np.clip(np.round(centroids[:, 0]).astype(np.int64), 0, H - 1)
        cc_i = np.clip(np.round(centroids[:, 1]).astype(np.int64), 0, W - 1)
        tri_cls_arr = self.class_10x[cr_i, cc_i]
        tri_classes = [(int(s[0]), int(s[1]), int(s[2]), int(c))
                       for s, c in zip(simplices, tri_cls_arr)]

        # 第二步：找出建筑裙边三角形和建筑边界顶点
        edge_count = {}
        for i0, i1, i2, cls_center in tri_classes:
            if cls_center != 40:
                continue
            for k in range(3):
                k1 = (k + 1) % 3
                idx = (i0, i1, i2)
                edge = tuple(sorted((idx[k], idx[k1])))
                edge_count[edge] = edge_count.get(edge, 0) + 1

        building_boundary_vertices = set()
        skirt_face_flags = []
        for i0, i1, i2, cls_center in tri_classes:
            if cls_center != 40:
                skirt_face_flags.append(False)
                continue
            is_skirt = False
            for k in range(3):
                k1 = (k + 1) % 3
                idx = (i0, i1, i2)
                edge = tuple(sorted((idx[k], idx[k1])))
                if edge_count.get(edge, 0) == 1:
                    is_skirt = True
                    building_boundary_vertices.add(idx[k])
                    building_boundary_vertices.add(idx[k1])
            skirt_face_flags.append(is_skirt)

        # 第三步：根据裙边最大距离计算建筑下沉深度图（边界处 depth=0，裙边深处 depth=1cm）
        skirt_max_dist = 0.0
        for (i0, i1, i2, cls_center), is_skirt in zip(tri_classes, skirt_face_flags):
            if cls_center != 40 or not is_skirt:
                continue
            for idx in (i0, i1, i2):
                r, c = int(round(self.pts[idx][0])), int(round(self.pts[idx][1]))
                r = np.clip(r, 0, H - 1)
                c = np.clip(c, 0, W - 1)
                skirt_max_dist = max(skirt_max_dist, self.building_dist_m[r, c])

        if skirt_max_dist > 1e-6:
            t = np.clip(self.building_dist_m / skirt_max_dist, 0.0, 1.0)
            self.building_depth_map = self.building_buffer_depth * (t * t * (3.0 - 2.0 * t))
            print(f"  建筑裙边最大深度: {self.building_buffer_depth:.3f} m, 裙边最大距离: {skirt_max_dist:.3f} m")
        else:
            self.building_depth_map = np.zeros_like(self.building_dist_m)

        # 第四步：生成共享顶点（向量化）
        r = np.clip(np.round(self.pts[:, 0]).astype(np.int64), 0, H - 1)
        c = np.clip(np.round(self.pts[:, 1]).astype(np.int64), 0, W - 1)
        x = (c / SCALE) * PX_M * self.ue_scale
        y = -((r / SCALE) * PX_M * self.ue_scale)
        x -= (W / SCALE) * PX_M * self.ue_scale * 0.5
        y += (H / SCALE) * PX_M * self.ue_scale * 0.5
        cls = self.class_10x[r, c]
        dem_h = self.dem_10x[r, c].copy()
        water_m = cls == 0
        bld_m = cls == 40
        if np.any(water_m):
            dem_h[water_m] -= self.water_depth_map[r[water_m], c[water_m]]
        if np.any(bld_m):
            dem_h[bld_m] -= self.building_depth_map[r[bld_m], c[bld_m]]
        z = dem_h * self.ue_scale
        u = c / (W - 1)
        v = r / (H - 1)
        self.verts = list(zip(x, y, z, u, v))

        # 统一朝向（向量化）
        p0 = self.pts[simplices[:, 0]]
        p1 = self.pts[simplices[:, 1]]
        p2 = self.pts[simplices[:, 2]]
        cross = (p1[:, 0] - p0[:, 0]) * (p2[:, 1] - p0[:, 1]) - (p1[:, 1] - p0[:, 1]) * (p2[:, 0] - p0[:, 0])
        needs_swap = cross < 0
        swapped = simplices.copy()
        swapped[needs_swap, 0], swapped[needs_swap, 1] = swapped[needs_swap, 1], swapped[needs_swap, 0]

        # 第五步：分配面到对应 OBJ（逻辑密集，保留循环）
        for tri_idx, ((i0, i1, i2, cls_center), is_skirt) in enumerate(zip(tri_classes, skirt_face_flags)):
            s0, s1, s2 = swapped[tri_idx]

            if cls_center == 0:
                # 水体：地面 OBJ 中凹包用深色泥沙
                self.ground_faces.append((s0, s1, s2, MAT_WATER_BED))
                # 生成蓝色水面盖子
                water_vis = []
                for idx in (s0, s1, s2):
                    x, y, _, u, v = self.verts[idx]
                    pr, pc = int(round(self.pts[idx][0])), int(round(self.pts[idx][1]))
                    pr = np.clip(pr, 0, H - 1)
                    pc = np.clip(pc, 0, W - 1)
                    z = self.dem_10x[pr, pc] * self.ue_scale
                    self.water_verts.append((x, y, z, u, v))
                    water_vis.append(len(self.water_verts) - 1)
                self.water_faces.append((water_vis[0], water_vis[1], water_vis[2], MAT_WATER))
            elif cls_center == 40:
                # 建筑 OBJ 包含整个建筑区域
                building_vis = []
                for idx in (s0, s1, s2):
                    x, y, _, u, v = self.verts[idx]
                    if idx in building_boundary_vertices:
                        # 边界顶点：与地面 OBJ 同高，确保严丝合缝
                        z = self.verts[idx][2]
                    else:
                        # 内部顶点：原始 DEM 高度，覆盖地面 OBJ
                        pr, pc = int(round(self.pts[idx][0])), int(round(self.pts[idx][1]))
                        pr = np.clip(pr, 0, H - 1)
                        pc = np.clip(pc, 0, W - 1)
                        z = self.dem_10x[pr, pc] * self.ue_scale
                    self.building_verts.append((x, y, z, u, v))
                    building_vis.append(len(self.building_verts) - 1)
                self.building_faces.append((building_vis[0], building_vis[1], building_vis[2], MAT_BUILDING))

                # 裙边三角形同时留在地面 OBJ，用建筑纹理
                if is_skirt:
                    self.ground_faces.append((s0, s1, s2, MAT_BUILDING_GROUND))
            else:
                mat = class_to_mat(cls_center)
                self.ground_faces.append((s0, s1, s2, mat))

        print(f"  地面顶点: {len(self.verts):,}, 地面三角形: {len(self.ground_faces):,}")
        print(f"  水面顶点: {len(self.water_verts):,}, 水面三角形: {len(self.water_faces):,}")
        print(f"  建筑顶点: {len(self.building_verts):,}, 建筑三角形: {len(self.building_faces):,}")
        print(f"  耗时 {time.time()-t0:.1f}s")

    # ═══════════════════════════════════════════════════════════════
    # 法线与导出
    # ═══════════════════════════════════════════════════════════════

    @staticmethod
    def _compute_normals(verts, faces):
        """按面法线加权平均计算顶点法线（NumPy 向量化）。"""
        verts_arr = np.array([[v[0], v[1], v[2]] for v in verts], dtype=np.float64)
        norms = np.zeros((len(verts_arr), 3), dtype=np.float64)
        faces_arr = np.array(faces, dtype=np.int64)
        i0, i1, i2 = faces_arr[:, 0], faces_arr[:, 1], faces_arr[:, 2]
        p0 = verts_arr[i0]
        p1 = verts_arr[i1]
        p2 = verts_arr[i2]
        fn = np.cross(p1 - p0, p2 - p0)
        nm = np.linalg.norm(fn, axis=1, keepdims=True)
        nm[nm < 1e-12] = 1.0
        fn /= nm
        np.add.at(norms, i0, fn)
        np.add.at(norms, i1, fn)
        np.add.at(norms, i2, fn)
        mag = np.linalg.norm(norms, axis=1, keepdims=True)
        mag[mag < 1e-12] = 1.0
        norms /= mag
        return norms

    def _write_obj(self, obj_path, mtl_path, verts, faces, mat_names, mat_colors):
        """导出一个 OBJ + MTL（批量字符串拼接优化）。"""
        obj_path = Path(obj_path)
        mtl_path = Path(mtl_path)
        obj_path.parent.mkdir(parents=True, exist_ok=True)
        norms = self._compute_normals(verts, faces)

        # MTL
        mtl_lines = []
        for name, col in zip(mat_names, mat_colors):
            mtl_lines.append(f"newmtl {name}\n")
            mtl_lines.append(f"Kd {col[0]:.4f} {col[1]:.4f} {col[2]:.4f}\n")
            mtl_lines.append("Ks 0.1 0.1 0.1\n")
            mtl_lines.append("Ns 32.0\n\n")
        mtl_path.write_text("".join(mtl_lines), encoding="utf-8")
        print(f"写入 MTL: {mtl_path}")

        # OBJ —— 批量构建字符串
        lines = []
        lines.append("# 自适应地形（Delaunay + 密集边界点）\n")
        lines.append(f"mtllib {mtl_path.name}\n")
        lines.append("o Terrain\n")

        for x, y, z, _, _ in verts:
            lines.append(f"v {x:.6f} {y:.6f} {z:.6f}\n")
        for n in norms:
            lines.append(f"vn {n[0]:.6f} {n[1]:.6f} {n[2]:.6f}\n")
        for _, _, _, u, v in verts:
            lines.append(f"vt {u:.6f} {v:.6f}\n")

        faces_by_mat = [[] for _ in mat_names]
        for f in faces:
            faces_by_mat[f[3]].append(f)

        for mat_id, name in enumerate(mat_names):
            if not faces_by_mat[mat_id]:
                continue
            lines.append(f"usemtl {name}\n")
            for i0, i1, i2, _ in faces_by_mat[mat_id]:
                lines.append(f"f {i0+1}/{i0+1}/{i0+1} {i1+1}/{i1+1}/{i1+1} {i2+1}/{i2+1}/{i2+1}\n")

        obj_path.write_text("".join(lines), encoding="utf-8")
        print(f"写入 OBJ: {obj_path} ({obj_path.stat().st_size/1e6:.1f} MB)")

    def export(self):
        """导出 terrain_ground.obj + terrain_water.obj + terrain_building.obj。"""
        out = Path(self.out_dir)
        ground_obj = out / "terrain_ground.obj"
        ground_mtl = out / "terrain_ground.mtl"
        water_obj = out / "terrain_water.obj"
        water_mtl = out / "terrain_water.mtl"
        building_obj = out / "terrain_building.obj"
        building_mtl = out / "terrain_building.mtl"

        self._write_obj(ground_obj, ground_mtl, self.verts, self.ground_faces,
                        MAT_NAMES, MAT_COLORS)
        self._write_obj(water_obj, water_mtl, self.water_verts, self.water_faces,
                        WATER_MAT_NAMES, WATER_MAT_COLORS)
        self._write_obj(building_obj, building_mtl, self.building_verts, self.building_faces,
                        BUILDING_MAT_NAMES, BUILDING_MAT_COLORS)

    def build(self):
        """完整地形构建流程。"""
        t0 = time.time()
        self.load()
        self.build_delaunay()
        self.build_mesh()
        self.export()
        print(f"总耗时: {time.time()-t0:.1f}s")


# ═══════════════════════════════════════════════════════════════
# 编排辅助
# ═══════════════════════════════════════════════════════════════

def _check_centerline_water_align(obj_path: Path, csv_path: Path, tol: float = 1.0):
    """校验中心线 CSV 是否落在 terrain_water.obj 顶点 AABB 内（防坐标漂移）。

    OBJ 文件内为右手系 y=-north；UE 导入后 y 翻转为 +south，中心线 CSV
    直接以 UE 世界坐标读入，因此 CSV 的 y 应对应 OBJ y 的取反区间。
    通过返回 True；任一文件缺失则跳过并返回 True。
    """
    if not obj_path.exists() or not csv_path.exists():
        print("坐标对齐校验: 跳过（缺少 terrain_water.obj 或中心线 CSV）")
        return True

    verts = []
    with open(obj_path, encoding="utf-8") as f:
        for line in f:
            if line.startswith("v "):
                parts = line.split()
                if len(parts) >= 4:
                    verts.append((float(parts[1]), float(parts[2]), float(parts[3])))
    if not verts:
        print("坐标对齐校验: 失败（terrain_water.obj 无顶点）")
        return False
    arr = np.array(verts)
    vx_min, vx_max = arr[:, 0].min(), arr[:, 0].max()
    vy_min, vy_max = arr[:, 1].min(), arr[:, 1].max()
    vz_min, vz_max = arr[:, 2].min(), arr[:, 2].max()
    # OBJ y=-north → UE y=+south 区间为 [-vy_max, -vy_min]
    south_lo, south_hi = -vy_max, -vy_min

    pat = re.compile(r"\(X=([-\d.]+),Y=([-\d.]+),Z=([-\d.]+)\)")
    pts = []
    with open(csv_path, encoding="utf-8") as f:
        for line in f:
            m = pat.search(line)
            if m:
                pts.append((float(m.group(1)), float(m.group(2)), float(m.group(3))))
    if not pts:
        print("坐标对齐校验: 失败（中心线 CSV 无有效点）")
        return False

    def inside(v, lo, hi):
        return lo - tol <= v <= hi + tol

    bad = [(x, y, z) for (x, y, z) in pts
           if not (inside(x, vx_min, vx_max) and inside(y, south_lo, south_hi)
                   and inside(z, vz_min, vz_max))]
    if not bad:
        print(f"坐标对齐校验: 通过（{len(pts)} 个中心线点均在 water OBJ AABB 内）")
        return True
    print(f"坐标对齐校验: 失败（{len(bad)}/{len(pts)} 个中心线点超出 water OBJ AABB，示例: {bad[:3]}）")
    return False


def _stream_job(name, argv, log_path, q):
    """启动一个子进程任务，实时转发其输出（带前缀），结束后回报结果。"""
    env = dict(os.environ)
    env["PYTHONUTF8"] = "1"
    env["PYTHONIOENCODING"] = "utf-8"
    try:
        proc = subprocess.Popen(
            argv,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
            bufsize=1,
            env=env,
        )
    except Exception as e:  # 启动失败也走失败通道，不中断其它任务
        print(f"[{name}] 启动失败: {e}")
        q.put((name, 1, str(log_path)))
        return

    try:
        with open(log_path, "w", encoding="utf-8") as log:
            for line in proc.stdout:
                log.write(line)
                print(f"[{name}] {line}", end="", flush=True)
    finally:
        rc = proc.wait()

    q.put((name, rc, str(log_path)))
    if rc == 0:
        print(f"[{name}] 完成（退出码 0）")
    else:
        print(f"[{name}] 失败（退出码 {rc}），完整输出见日志: {log_path}")


def run_terrain_stage(label_10x_path, boundary_path, dem_path, out_dir, boundary_step):
    """独立运行地形构建（供 --stage-terrain 与并行编排调用）。"""
    print("=" * 60)
    print("地形 OBJ 构建（AdaptiveTerrainBuilder）")
    print("=" * 60)
    builder = AdaptiveTerrainBuilder(
        label_10x_path=label_10x_path,
        boundary_path=boundary_path,
        dem_path=dem_path,
        out_dir=out_dir,
        boundary_sample_step=boundary_step,
    )
    builder.build()


# ═══════════════════════════════════════════════════════════════
# 完整管线入口（编排）
# ═══════════════════════════════════════════════════════════════

def _parse_regenerate(values):
    """解析 --regenerate 列表（支持逗号分隔与重复指定）。"""
    allowed = {"classify", "postprocess", "semantic", "terrain", "flow"}
    parts = []
    for v in values or []:
        parts.extend(x.strip() for x in v.split(",") if x.strip())
    for x in parts:
        if x not in allowed:
            raise SystemExit(f"--regenerate 取值无效: {x}（可选: {', '.join(sorted(allowed))}）")
    return list(dict.fromkeys(parts))  # 去重保序


def main():
    p = argparse.ArgumentParser(description="3DMapRebuilder 全管线编排 + 自适应地形生成器")
    p.add_argument("--label-dir", default="TestInput/SanHe", help="labels.tif 所在目录")
    p.add_argument("--dem", default="TestInput/SanHe/dem.tif", help="DEM 路径")
    p.add_argument("--out-dir", default="output/terrain_adaptive", help="地形/流场输出目录")
    p.add_argument("--boundary-step", type=int, default=BOUNDARY_SAMPLE_STEP, help="边界采样步长（10x 像素）")
    p.add_argument("--velocity-res", type=int, default=2048, help="速度场纹理最长边像素数")
    p.add_argument("--velocity-seed", type=int, default=42, help="Perlin 噪声随机种子")
    p.add_argument("--velocity-noise", type=float, default=0.15, help="流向噪声强度（弧度）")
    p.add_argument("--force", action="store_true",
                   help="强制重新生成所有产物（含已有中间文件）")
    p.add_argument("--regenerate", action="append", default=[],
                   help="只强制重新生成指定部分（classify/postprocess/semantic/terrain/flow，"
                        "可重复指定或以逗号分隔，如 --regenerate terrain,flow）；"
                        "其余部分仍按缓存逻辑（产物存在则跳过）")
    p.add_argument("--stage-terrain", action="store_true",
                   help=argparse.SUPPRESS)  # 内部编排用：只跑地形构建
    args = p.parse_args()
    regen = _parse_regenerate(args.regenerate)
    force = args.force

    label_dir = Path(args.label_dir)
    dem_path = str(Path(args.dem))
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    log_dir = out_dir / "_logs"
    log_dir.mkdir(parents=True, exist_ok=True)

    label_10x_path = label_dir / "Output_10x" / "labels_10x_no_boundary.tif"
    boundary_path = label_dir / "Output_10x" / "boundary_lines_10x.png"
    out_10x = label_dir / "Output_10x"

    # 仅地形模式（供并行编排内部调用）：不触发分类/后处理/并行编排
    if args.stage_terrain:
        run_terrain_stage(
            str(label_10x_path), str(boundary_path), dem_path,
            str(out_dir), args.boundary_step,
        )
        return

    label_path = label_dir / "labels.tif"
    semantic_ply = label_dir / "semanticMap.ply"
    obj_paths = [out_dir / "terrain_ground.obj", out_dir / "terrain_water.obj",
                 out_dir / "terrain_building.obj"]
    flow_paths = [out_dir / "terrain_water_centerline.csv",
                  out_dir / "terrain_velocity_field.png"]

    def force_part(part):
        return force or part in regen

    # ── 1. 语义分类 ──
    if not force_part("classify") and label_path.exists():
        print(f"使用已有: {label_path}")
    else:
        print("=" * 60)
        print("阶段 1: 语义分类（classify_vecw）")
        print("=" * 60)
        clf = VecClassifier()
        clf.set_input(str(label_dir / "vec_raw.png"), str(label_dir / "satellite.tif"))
        clf.set_output(str(label_dir))
        clf.run()

    # ── 2. 标签后处理 ──
    if not force_part("postprocess") and label_10x_path.exists() and boundary_path.exists():
        print(f"使用已有: {label_10x_path}, {boundary_path}")
    else:
        print("=" * 60)
        print("阶段 2: 标签后处理（label_postprocess）")
        print("=" * 60)
        pp = LabelPostprocessor(input_path=str(label_path), output_dir=str(out_10x))
        pp.process()

    # ── 3. 三路并行：3DGS 语义地图 / 地形 OBJ / 水体流场 ──
    jobs = []
    if not force_part("semantic") and semantic_ply.exists():
        print(f"使用已有: {semantic_ply}")
    else:
        jobs.append((
            "semantic",
            [sys.executable, str(Path(__file__).parent / "gen_semantic.py"),
             "--label", str(label_path), "--dem", dem_path, "--out", str(semantic_ply)],
        ))
    if not force_part("terrain") and all(p.exists() for p in obj_paths):
        print(f"使用已有: 地形 OBJ（{out_dir}）")
    else:
        jobs.append((
            "terrain",
            [sys.executable, str(Path(__file__).resolve()),
             "--stage-terrain", "--label-dir", str(label_dir),
             "--dem", dem_path, "--out-dir", str(out_dir),
             "--boundary-step", str(args.boundary_step)],
        ))
    if not force_part("flow") and all(p.exists() for p in flow_paths):
        print(f"使用已有: 中心线 CSV + 速度场纹理（{out_dir}）")
    else:
        jobs.append((
            "flow",
            [sys.executable, str(Path(__file__).parent / "gen_water_flow.py"),
             "--label-10x", str(label_10x_path), "--dem", dem_path,
             "--out-dir", str(out_dir),
             "--velocity-res", str(args.velocity_res),
             "--velocity-seed", str(args.velocity_seed),
             "--velocity-noise", str(args.velocity_noise)],
        ))

    failed = []
    if jobs:
        print("=" * 60)
        print("并行阶段: " + " / ".join(job[0] for job in jobs))
        print("（各任务独立运行，单个失败不影响其它任务继续）")
        print("=" * 60)
        q = queue.Queue()
        threads = []
        for name, argv in jobs:
            t = threading.Thread(
                target=_stream_job, args=(name, argv, log_dir / f"{name}.log", q),
                daemon=True,
            )
            threads.append(t)
            t.start()
        results = [q.get() for _ in threads]
        for t in threads:
            t.join()
        failed = [(n, rc, lp) for (n, rc, lp) in results if rc != 0]
        print("=" * 60)
        print(f"并行阶段结束: {len(results) - len(failed)}/{len(results)} 成功")
        for n, rc, lp in failed:
            print(f"  [{n}] 失败（退出码 {rc}），日志: {lp}")
    else:
        print("所有产物均已存在，无需执行并行阶段")

    # ── 4. 坐标对齐校验（中心线 CSV vs 水面 OBJ）──
    align_ok = _check_centerline_water_align(out_dir / "terrain_water.obj",
                                             out_dir / "terrain_water_centerline.csv")

    exit_code = 1 if (failed or not align_ok) else 0
    if exit_code == 0:
        print(f"全部完成，输出目录: {out_dir}")
    sys.exit(exit_code)


def test():
    """完整管线测试入口（默认重新生成所有文件）。"""
    main()


if __name__ == "__main__":
    main()

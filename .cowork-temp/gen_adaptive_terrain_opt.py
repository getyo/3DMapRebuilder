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

import argparse
import os
import sys
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
BUILDING_BUFFER_DEPTH = 0.01     # 建筑裙边最大下沉深度（米）
PX_M = 0.458                     # 原始像素地面尺寸（米）
UE_SCALE = 100.0                 # UE 单位缩放

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
# 自适应地形构建器
# ═══════════════════════════════════════════════════════════════

class AdaptiveTerrainBuilder:
    """
    从 10x 语义标签、边界线图和 DEM 生成 UE 可用的自适应分辨率地形网格。
    输出三个 OBJ：
      - terrain_ground.obj：地面/道路/水体凹包 + 建筑裙边
      - terrain_building.obj：建筑核心区域顶面
      - terrain_water.obj：蓝色水面盖子
    三个 OBJ 同坐标系，UE 000 对齐。
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
    # 网格生成（核心向量化优化）
    # ═══════════════════════════════════════════════════════════════

    def build_mesh(self):
        """根据 Delaunay 结果生成地面网格、建筑网格和水面盖子网格。"""
        print("构建三维网格...")
        t0 = time.time()
        H, W = self.H10, self.W10
        simplices = self.tri.simplices  # (N, 3)
        N = len(simplices)

        # 第一步：收集每个三角形的重心 class（NumPy 向量化）
        pts_tri = self.pts[simplices]  # (N, 3, 2)
        centroids = pts_tri.mean(axis=1)  # (N, 2)
        cr_i = np.clip(np.round(centroids[:, 0]).astype(np.int64), 0, H - 1)
        cc_i = np.clip(np.round(centroids[:, 1]).astype(np.int64), 0, W - 1)
        tri_classes = self.class_10x[cr_i, cc_i]  # (N,)

        # 第二步：找出建筑裙边三角形和建筑边界顶点（向量化）
        edges = np.concatenate([
            np.sort(simplices[:, [0, 1]], axis=1),
            np.sort(simplices[:, [1, 2]], axis=1),
            np.sort(simplices[:, [2, 0]], axis=1),
        ], axis=0)
        tri_idx = np.repeat(np.arange(N), 3)

        building_tri_mask = tri_classes == 40
        building_edge_mask = np.repeat(building_tri_mask, 3)
        building_edges = edges[building_edge_mask]

        building_boundary_vertices = set()
        skirt_face_flags = np.zeros(N, dtype=bool)
        if len(building_edges) > 0:
            be_view = building_edges.copy().view(np.dtype((np.void, building_edges.dtype.itemsize * 2))).ravel()
            _, inv, counts = np.unique(be_view, return_inverse=True, return_counts=True)
            boundary_edge_mask = counts[inv] == 1
            boundary_edges = building_edges[boundary_edge_mask]
            if len(boundary_edges) > 0:
                building_boundary_vertices = set(np.unique(boundary_edges.ravel()))

                # 向量化：找到包含边界边的三角形
                all_edges_sorted = np.sort(edges, axis=1)
                dt = np.dtype((np.void, all_edges_sorted.dtype.itemsize * 2))
                all_edges_view = all_edges_sorted.copy().view(dt).ravel()
                boundary_edges_view = boundary_edges.copy().view(dt).ravel()
                is_boundary_edge = np.isin(all_edges_view, boundary_edges_view)
                boundary_tri_indices = tri_idx[is_boundary_edge]
                if len(boundary_tri_indices) > 0:
                    tri_has_boundary_edge = np.zeros(N, dtype=bool)
                    tri_has_boundary_edge[np.unique(boundary_tri_indices)] = True
                    skirt_face_flags = building_tri_mask & tri_has_boundary_edge

        # 第三步：根据裙边最大距离计算建筑下沉深度图
        skirt_max_dist = 0.0
        if np.any(skirt_face_flags):
            skirt_tris = simplices[skirt_face_flags]
            skirt_vert_indices = np.unique(skirt_tris.ravel())
            skirt_r = np.clip(np.round(self.pts[skirt_vert_indices, 0]).astype(np.int64), 0, H - 1)
            skirt_c = np.clip(np.round(self.pts[skirt_vert_indices, 1]).astype(np.int64), 0, W - 1)
            skirt_max_dist = self.building_dist_m[skirt_r, skirt_c].max()

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
        water_mask_pts = cls == 0
        bld_mask_pts = cls == 40
        if np.any(water_mask_pts):
            dem_h[water_mask_pts] -= self.water_depth_map[r[water_mask_pts], c[water_mask_pts]]
        if np.any(bld_mask_pts):
            dem_h[bld_mask_pts] -= self.building_depth_map[r[bld_mask_pts], c[bld_mask_pts]]
        z = dem_h * self.ue_scale
        u = c / (W - 1)
        v = r / (H - 1)
        self.verts = list(zip(x, y, z, u, v))
        verts_arr = np.array(self.verts, dtype=np.float64)

        # 第五步：统一朝向（向量化）
        p0 = self.pts[simplices[:, 0]]
        p1 = self.pts[simplices[:, 1]]
        p2 = self.pts[simplices[:, 2]]
        cross = (p1[:, 0] - p0[:, 0]) * (p2[:, 1] - p0[:, 1]) - (p1[:, 1] - p0[:, 1]) * (p2[:, 0] - p0[:, 0])
        needs_swap = cross < 0
        i0 = simplices[:, 0].copy()
        i1 = simplices[:, 1].copy()
        i2 = simplices[:, 2].copy()
        i0[needs_swap], i1[needs_swap] = i1[needs_swap], i0[needs_swap]

        # 第六步：分配面到对应 OBJ
        water_mask = tri_classes == 0
        building_mask = tri_classes == 40
        ground_mask = (tri_classes != 40) | skirt_face_flags

        bd_set = building_boundary_vertices

        # --- 水面 OBJ ---
        if np.any(water_mask):
            water_tris = np.column_stack([i0[water_mask], i1[water_mask], i2[water_mask]])
            water_vert_idx = water_tris.ravel()
            water_xyuv = verts_arr[water_vert_idx][:, [0, 1, 3, 4]]
            water_r = np.clip(np.round(self.pts[water_vert_idx, 0]).astype(np.int64), 0, H - 1)
            water_c = np.clip(np.round(self.pts[water_vert_idx, 1]).astype(np.int64), 0, W - 1)
            water_z = self.dem_10x[water_r, water_c] * self.ue_scale
            water_verts_arr = np.column_stack([water_xyuv[:, 0], water_xyuv[:, 1], water_z, water_xyuv[:, 2], water_xyuv[:, 3]])
            self.water_verts = [tuple(v) for v in water_verts_arr]
            water_faces_idx = np.arange(len(water_vert_idx)).reshape(-1, 3)
            self.water_faces = [(int(a), int(b), int(c), MAT_WATER) for a, b, c in water_faces_idx]
        else:
            self.water_verts = []
            self.water_faces = []

        # --- 建筑 OBJ ---
        if np.any(building_mask):
            building_tris = np.column_stack([i0[building_mask], i1[building_mask], i2[building_mask]])
            building_vert_idx = building_tris.ravel()
            is_bd = np.isin(building_vert_idx, list(bd_set))
            building_xyuv = verts_arr[building_vert_idx][:, [0, 1, 3, 4]]
            building_r = np.clip(np.round(self.pts[building_vert_idx, 0]).astype(np.int64), 0, H - 1)
            building_c = np.clip(np.round(self.pts[building_vert_idx, 1]).astype(np.int64), 0, W - 1)
            building_z = self.dem_10x[building_r, building_c] * self.ue_scale
            if np.any(is_bd):
                building_z[is_bd] = verts_arr[building_vert_idx[is_bd], 2]
            building_verts_arr = np.column_stack([building_xyuv[:, 0], building_xyuv[:, 1], building_z, building_xyuv[:, 2], building_xyuv[:, 3]])
            self.building_verts = [tuple(v) for v in building_verts_arr]
            building_faces_idx = np.arange(len(building_vert_idx)).reshape(-1, 3)
            self.building_faces = [(int(a), int(b), int(c), MAT_BUILDING) for a, b, c in building_faces_idx]
        else:
            self.building_verts = []
            self.building_faces = []

        # --- 地面 OBJ ---
        if np.any(ground_mask):
            ground_tri_indices = np.where(ground_mask)[0]
            ground_tris = np.column_stack([i0[ground_tri_indices], i1[ground_tri_indices], i2[ground_tri_indices]])
            ground_classes = tri_classes[ground_tri_indices]
            ground_mats = class_to_mat(ground_classes)
            # 裙边三角形覆盖为建筑材质
            ground_mats[skirt_face_flags[ground_tri_indices]] = MAT_BUILDING_GROUND
            self.ground_faces = [(int(a), int(b), int(c), int(m)) for a, b, c, m in zip(ground_tris[:, 0], ground_tris[:, 1], ground_tris[:, 2], ground_mats)]
        else:
            self.ground_faces = []

        print(f"  地面顶点: {len(self.verts):,}, 地面三角形: {len(self.ground_faces):,}")
        print(f"  水面顶点: {len(self.water_verts):,}, 水面三角形: {len(self.water_faces):,}")
        print(f"  建筑顶点: {len(self.building_verts):,}, 建筑三角形: {len(self.building_faces):,}")
        print(f"  耗时 {time.time()-t0:.1f}s")

    # ═══════════════════════════════════════════════════════════════
    # 法线与导出（向量化优化）
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

    label_dir = Path(args.label_dir)
    label_path = str(label_dir / "labels.tif")
    out_10x = label_dir / "Output_10x"
    label_10x_path = str(out_10x / "labels_10x_no_boundary.tif")
    boundary_path = str(out_10x / "boundary_lines_10x.png")
    semantic_ply = str(label_dir / "semanticMap.ply")

    vec_path = str(label_dir / "vec_raw.png")
    sate_path = str(label_dir / "satellite.tif")

    # 1. 语义分类
    if not args.force and Path(label_path).exists():
        print(f"使用已有: {label_path}")
    else:
        print("=" * 60)
        print("Stage 1/4: 语义分类")
        print("=" * 60)
        clf = VecClassifier()
        clf.set_input(vec_path, sate_path)
        clf.set_output(str(label_dir))
        clf.run()

    # 2. 标签后处理
    if not args.force and Path(label_10x_path).exists() and Path(boundary_path).exists():
        print(f"使用已有: {label_10x_path}, {boundary_path}")
    else:
        print("=" * 60)
        print("Stage 2/4: 标签后处理")
        print("=" * 60)
        pp = LabelPostprocessor(input_path=label_path, output_dir=str(out_10x))
        pp.process()

    # 3. 简化 3DGS 语义地图
    if not args.force and Path(semantic_ply).exists():
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

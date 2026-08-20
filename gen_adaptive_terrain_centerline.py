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
import sys
import time
from pathlib import Path

import numpy as np
import rasterio
import cv2
import networkx as nx
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
        self.velocity_res = 2048

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
        self.centerline_points = None  # 中心线世界坐标（N, 3）

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
    # 水体中心线提取
    # ═══════════════════════════════════════════════════════════════

    def _extract_centerline(self):
        """从 water_mask 提取河流中心线，输出 CSV 与预览图。"""
        from skimage.morphology import skeletonize
        from scipy.ndimage import gaussian_filter1d

        if self.water_mask is None or not np.any(self.water_mask):
            print("  无水体，跳过中心线提取")
            return

        print("提取水体中心线...")
        t0 = time.time()
        H, W = self.water_mask.shape

        # 1. 下采样后骨架化（避免大分辨率全图骨架化卡死）
        target_skeleton_size = 2048
        max_side = max(H, W)
        scale_factor = target_skeleton_size / max_side
        if scale_factor >= 1.0:
            water_small = self.water_mask
            scale_factor = 1.0
        else:
            H_small = int(H * scale_factor)
            W_small = int(W * scale_factor)
            water_small = cv2.resize(
                self.water_mask.astype(np.uint8) * 255,
                (W_small, H_small),
                interpolation=cv2.INTER_NEAREST
            ) > 0

        skel = skeletonize(water_small)
        if not np.any(skel):
            print("  骨架为空，跳过")
            return

        # 2. 8-邻接建图
        Hs, Ws = water_small.shape
        G = nx.Graph()
        for r, c in zip(*np.where(skel)):
            node = (int(r), int(c))
            G.add_node(node)
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1),
                           (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                nr, nc = r + dr, c + dc
                if 0 <= nr < Hs and 0 <= nc < Ws and skel[nr, nc]:
                    G.add_edge(node, (int(nr), int(nc)))

        if G.number_of_nodes() == 0:
            print("  骨架图为空，跳过")
            return

        # 3. 取最大连通分量并找直径（两次 BFS）
        components = list(nx.connected_components(G))
        largest = max(components, key=len)
        G_main = G.subgraph(largest).copy()

        def bfs_farthest(start):
            lengths = nx.single_source_shortest_path_length(G_main, start)
            far_node = max(lengths, key=lengths.get)
            return far_node, lengths

        start = next(iter(G_main.nodes))
        A, _ = bfs_farthest(start)
        B, _ = bfs_farthest(A)
        path = nx.shortest_path(G_main, A, B)

        # 4. 映射回原始分辨率并平滑
        path_full = [(r / scale_factor, c / scale_factor) for r, c in path]
        path_arr = np.array(path_full, dtype=float)
        sigma = 3.0
        r_smooth = gaussian_filter1d(path_arr[:, 0], sigma=sigma)
        c_smooth = gaussian_filter1d(path_arr[:, 1], sigma=sigma)

        # 5. 按弧长重采样（默认 500cm UE 间距）
        sample_spacing_cm = 500.0
        sample_spacing_px = sample_spacing_cm / ((PX_M / SCALE) * UE_SCALE)
        diffs = np.diff(np.stack([r_smooth, c_smooth], axis=1), axis=0)
        arc = np.cumsum(np.linalg.norm(diffs, axis=1))
        arc = np.insert(arc, 0, 0.0)
        total_arc = arc[-1]
        n_samples = max(2, int(np.ceil(total_arc / sample_spacing_px)) + 1)
        sample_arc = np.linspace(0.0, total_arc, n_samples)

        r_sampled = np.interp(sample_arc, arc, r_smooth)
        c_sampled = np.interp(sample_arc, arc, c_smooth)

        # 6. 像素坐标 → UE 世界坐标
        x = (c_sampled / SCALE) * PX_M * self.ue_scale
        y = -((r_sampled / SCALE) * PX_M * self.ue_scale)
        x -= (W / SCALE) * PX_M * self.ue_scale * 0.5
        y += (H / SCALE) * PX_M * self.ue_scale * 0.5

        r_i = np.clip(np.round(r_sampled).astype(np.int64), 0, H - 1)
        c_i = np.clip(np.round(c_sampled).astype(np.int64), 0, W - 1)
        z = self.dem_10x[r_i, c_i] * self.ue_scale

        points = [{"x": float(x[i]), "y": float(y[i]), "z": float(z[i])}
                  for i in range(len(x))]
        self.centerline_points = np.column_stack([x, y, z])

        # 7. 输出 CSV（UE DataTable 格式，结构体含 FVector Position）
        out = Path(self.out_dir)
        out.mkdir(parents=True, exist_ok=True)
        csv_path = out / "terrain_water_centerline.csv"
        lines = ["---,Position"]
        for i, p in enumerate(points):
            pos = f"\"(X={p['x']:.6f},Y={p['y']:.6f},Z={p['z']:.6f})\""
            lines.append(f"Point{i},{pos}")
        csv_path.write_text("\n".join(lines), encoding="utf-8")
        print(f"  写入中心线 CSV: {csv_path}")

        # 8. 输出预览图（不覆盖原有水体掩膜输出）
        preview = np.zeros((H, W, 3), dtype=np.uint8)
        # 水体掩膜用淡蓝（BGR：蓝=255，绿=210，红=180）
        preview[self.water_mask] = [255, 210, 180]
        rr = np.clip(np.round(r_sampled).astype(np.int64), 0, H - 1)
        cc = np.clip(np.round(c_sampled).astype(np.int64), 0, W - 1)
        for i in range(len(rr) - 1):
            cv2.line(preview, (int(cc[i]), int(rr[i])),
                     (int(cc[i + 1]), int(rr[i + 1])), (0, 0, 255), thickness=3)
        preview_path = out / "terrain_water_centerline_preview.png"
        cv2.imwrite(str(preview_path), preview)
        print(f"  写入中心线预览: {preview_path}")

        print(f"  中心线提取耗时 {time.time()-t0:.1f}s")

    def _generate_velocity_field(self, resolution: int = 2048):
        """以中心线为骨架生成水体静态速度场纹理。"""
        if self.centerline_points is None or len(self.centerline_points) < 2:
            print("  中心线缺失，跳过速度场生成")
            return
        if not self.water_verts:
            print("  水面网格为空，跳过速度场生成")
            return

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
        """完整构建流程。"""
        t0 = time.time()
        self.load()
        self.build_delaunay()
        self.build_mesh()
        self._extract_centerline()
        self._generate_velocity_field(resolution=self.velocity_res)
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
    p.add_argument("--velocity-res", type=int, default=2048, help="速度场纹理最长边像素数")
    p.add_argument("--force", action="store_true",
                   help="强制重新生成所有中间文件；默认会利用已有文件")
    args = p.parse_args()

    label_dir = Path(args.label_dir)
    label_path = str(label_dir / "labels.tif")
    out_10x = str(label_dir / "Output_10x")
    label_10x_path = str(label_dir / "Output_10x" / "labels_10x_no_boundary.tif")
    boundary_path = str(label_dir / "Output_10x" / "boundary_lines_10x.png")
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
        pp = LabelPostprocessor(input_path=label_path, output_dir=out_10x)
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
    terrain_builder.velocity_res = args.velocity_res
    terrain_builder.build()


def test():
    """完整管线测试入口（默认重新生成所有文件）。"""
    main()


if __name__ == "__main__":
    main()

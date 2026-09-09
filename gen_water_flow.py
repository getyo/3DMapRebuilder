#!/usr/bin/env python3
"""
水体流场生成器（河网分解 + 分流速度场）— 独立模块

输入：
  - labels_10x_no_boundary.tif   10x 语义标签（水体 = band1 == 0）
  - dem.tif                      高程（1x 分辨率，内部升采样到 10x）

输出（写入 out_dir）：
  - terrain_water_centerline.csv              主流中心线（UE DataTable 格式）
  - terrain_water_centerline_preview.png      河网预览图（主流红 + 分流按速度衰减着色）
  - terrain_velocity_field.png                静态速度场纹理（RGBA）
  - terrain_velocity_field_preview.png        速度场 HSV 预览图

依赖关系：
  水体掩膜必须来自 label_postprocess 平滑后的 10x 标签，保证与
  terrain_water.obj 的水体区域一致；本模块不依赖 Delaunay / 地形网格，
  可与地形 OBJ、3DGS 语义地图并行生成（主流程见
  gen_adaptive_terrain_centerline.py 的三路并行编排）。

河网模型（分流语义）：
  1. 水体掩膜下采样后骨架化，得到河网图；对每个连通分量：
  2. 主流 = 该分量的直径路径（最长最短路径，方向 A→B），主流速度因子恒为 1。
  3. 主流之外的边构成"分流树"，每条分流树以离 A 最近的汇口为根，方向从
     汇口指向自身末端（水从主流分流出去）。
  4. 分流内每个节点的速度因子 = max(residual, (1 - d/L)^α)，
     其中 d 为该节点离汇口的图上距离，L 为该分流树的最大离汇距离，
     α = velocity_branch_alpha，residual = velocity_branch_residual。
  5. 主干上长出的分流树若有两条与主干相接（汊道环），按主流方向赋值、
     速度因子恒为 1，不做衰减。
  6. 独立的多个水体各自跑一遍上述流程，最终合并进同一张纹理。

坐标与纹理约定（常量统一见 map_common.py）：
  - 中心线 CSV 的 y 为 UE 左手系 y=+south，与 UE 导入 OBJ 后的 mesh 一致。
  - 速度场纹理空间 = 整幅 10x 图像空间，V-up 布局，可直接按水面 mesh 的
    UV 采样（UV AABB 为 [0,1]×[0,1]）。
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import rasterio
import cv2
import networkx as nx

# 添加脚本目录到导入路径，确保 standalone 运行能找到同级模块
sys.path.insert(0, str(Path(__file__).parent))
from map_common import (UE_SCALE, UE_PER_PX10,
                        ue_x_of_col, ue_y_csv_of_row)


class WaterFlowBuilder:
    """
    从 10x 水体掩膜 + DEM 提取河网（主流 + 分流）并生成静态速度场纹理。

    用法：
      b = WaterFlowBuilder(label_10x_path='...', dem_path='...', out_dir='...')
      b.build()
    """

    def __init__(
        self,
        label_10x_path: str,
        dem_path: str,
        out_dir: str,
        velocity_res: int = 2048,
        velocity_seed: int = 42,
        velocity_noise_scale: float = 0.15,
        velocity_branch_alpha: float = 0.5,
        velocity_branch_residual: float = 0.1,
        velocity_converge: float = 0.0,
    ):
        self.label_10x_path = label_10x_path
        self.dem_path = dem_path
        self.out_dir = out_dir
        # 速度场参数
        self.velocity_res = velocity_res              # 纹理最长边像素数
        self.velocity_seed = velocity_seed            # Perlin 噪声随机种子
        self.velocity_noise_scale = velocity_noise_scale  # 流向噪声强度（弧度）
        self.velocity_branch_alpha = velocity_branch_alpha      # 分流衰减幂次 α
        self.velocity_branch_residual = velocity_branch_residual  # 分流末端残余速度因子
        self.velocity_converge = velocity_converge                # 向心偏转强度 k（0 = 关闭）

        self.H10 = self.W10 = 0
        self.water_mask = None
        self.dem_10x = None
        self.centerline_points = None   # 主流中心线世界坐标（N, 3）：x=east, y=+south, z=height

        # 河网（骨架分辨率坐标）
        self.water_small = None         # 下采样后的水体掩膜（骨架分辨率）
        self.net_r = None               # (N,) 骨架节点行坐标
        self.net_c = None               # (N,) 骨架节点列坐标
        self.net_tan = None             # (N, 2) 节点切向（沿流向单位向量）
        self.net_factor = None          # (N,) 节点速度因子
        self.net_half = None            # (N,) 节点局部半宽（像素）

    # ═══════════════════════════════════════════════════════════════
    # 加载
    # ═══════════════════════════════════════════════════════════════

    def load(self):
        """加载 10x 标签与 DEM，提取水体掩膜。"""
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
        if np.any(self.water_mask):
            print(f"  水体像素: {self.water_mask.sum():,}")
        else:
            print("  无水体像素")

    # ═══════════════════════════════════════════════════════════════
    # 河网提取
    # ═══════════════════════════════════════════════════════════════

    @staticmethod
    def _factor_color(factor):
        """factor ∈ [0,1] → JET 色（低=蓝，高=红），用于预览着色。"""
        v = int(round(np.clip(factor, 0.0, 1.0) * 255))
        return cv2.applyColorMap(np.array([[v]], dtype=np.uint8), cv2.COLORMAP_JET)[0, 0]

    def _extract_network(self):
        """骨架化 + 河网分解，计算每个骨架节点的切向/速度因子/半宽，写 CSV 与预览。"""
        from skimage.morphology import skeletonize
        from scipy.ndimage import gaussian_filter1d, distance_transform_edt

        if self.water_mask is None or not np.any(self.water_mask):
            print("  无水体，跳过河网提取")
            return

        print("提取河网（主流 + 分流）...")
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
        self.water_small = water_small
        self._scale_factor = scale_factor

        skel = skeletonize(water_small)
        if not np.any(skel):
            print("  骨架为空，跳过")
            return

        Hs, Ws = water_small.shape
        # 局部半宽 = 骨架点到最近岸边的距离（骨架即中轴）
        dist_bank = distance_transform_edt(water_small)

        # 2. 8-邻接建图
        G = nx.Graph()
        skel_nodes = list(zip(*np.where(skel)))
        for r, c in skel_nodes:
            node = (int(r), int(c))
            G.add_node(node)
        for r, c in skel_nodes:
            node = (int(r), int(c))
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1),
                           (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                nr, nc = r + dr, c + dc
                if 0 <= nr < Hs and 0 <= nc < Ws and skel[nr, nc]:
                    G.add_edge(node, (int(nr), int(nc)))

        if G.number_of_nodes() == 0:
            print("  骨架图为空，跳过")
            return

        # 3. 按连通分量处理（独立水体各自成网）
        components = sorted(nx.connected_components(G), key=len, reverse=True)

        net_r, net_c, net_tan, net_factor, net_half = [], [], [], [], []
        branch_segs = []   # (r1,c1,r2,c2,factor) 供预览画分流
        trunk_skel = None  # 最大分量主流骨架路径（供 CSV/预览）

        for ci, comp in enumerate(components):
            sub = G.subgraph(comp).copy()

            def bfs_farthest(start):
                lengths = nx.single_source_shortest_path_length(sub, start)
                far_node = max(lengths, key=lengths.get)
                return far_node, lengths

            start = next(iter(sub.nodes))
            A, _ = bfs_farthest(start)
            B, _ = bfs_farthest(A)
            trunk_path = nx.shortest_path(sub, A, B)   # 有序节点列表 A→B
            trunk_set = set(trunk_path)
            trunk_index = {n: i for i, n in enumerate(trunk_path)}

            # 主流切向（骨架坐标）
            tp = np.array(trunk_path, dtype=np.float64)  # (Np, 2) [r, c]
            t_tan = np.zeros_like(tp)
            if len(tp) >= 2:
                t_tan[1:-1] = tp[2:] - tp[:-2]
                t_tan[0] = tp[1] - tp[0]
                t_tan[-1] = tp[-1] - tp[-2]
            tnorm = np.linalg.norm(t_tan, axis=1, keepdims=True)
            tnorm[tnorm < 1e-8] = 1.0
            t_tan = t_tan / tnorm

            for i, (r, c) in enumerate(trunk_path):
                net_r.append(r)
                net_c.append(c)
                net_tan.append(t_tan[i].astype(np.float32))
                net_factor.append(1.0)
                net_half.append(max(float(dist_bank[r, c]), 0.5))

            # 分流树 = 去掉主流边后的连通分量
            trunk_edges = {frozenset((trunk_path[i], trunk_path[i + 1]))
                           for i in range(len(trunk_path) - 1)}
            branch_edges = [e for e in sub.edges() if frozenset(e) not in trunk_edges]
            if branch_edges:
                branch_graph = nx.Graph()
                branch_graph.add_edges_from(branch_edges)
                for bcomp in nx.connected_components(branch_graph):
                    bsub = branch_graph.subgraph(bcomp).copy()
                    b_trunk_nodes = [n for n in bcomp if n in trunk_set]
                    non_trunk = [n for n in bcomp if n not in trunk_set]
                    if not non_trunk:
                        continue
                    # 根 = 离 A 最近的汇口
                    if b_trunk_nodes:
                        root = min(b_trunk_nodes, key=lambda n: trunk_index[n])
                    else:
                        root = next(iter(bcomp))
                    pred = dict(nx.bfs_predecessors(bsub, root))
                    dist = nx.single_source_shortest_path_length(bsub, root)

                    # 汊道环：两端都接主流且无悬空末端 → 因子恒 1
                    is_anabranch = (len(b_trunk_nodes) >= 2 and
                                    all(bsub.degree(n) != 1 for n in non_trunk))
                    if is_anabranch:
                        L_max = 1.0
                    else:
                        L_max = max((dist[n] for n in non_trunk if n in dist), default=0.0)
                        if L_max <= 0:
                            L_max = 1.0

                    for n in non_trunk:
                        d = dist.get(n, 0)
                        if is_anabranch:
                            fac = 1.0
                        else:
                            fac = max(self.velocity_branch_residual,
                                      (1.0 - d / L_max) ** self.velocity_branch_alpha)
                        net_r.append(n[0])
                        net_c.append(n[1])
                        p = pred.get(n)
                        if p is not None:
                            vec = np.array(n, dtype=np.float32) - np.array(p, dtype=np.float32)
                        else:
                            vec = np.array([0.0, 0.0], dtype=np.float32)
                        vn = np.linalg.norm(vec)
                        vec = vec / vn if vn > 1e-8 else vec
                        net_tan.append(vec.astype(np.float32))
                        net_factor.append(float(fac))
                        net_half.append(max(float(dist_bank[n[0], n[1]]), 0.5))
                        branch_segs.append((p[0], p[1], n[0], n[1], float(fac)) if p is not None
                                           else (n[0], n[1], n[0], n[1], float(fac)))

            # 最大分量：记录主流骨架路径，用于 CSV 与预览
            if trunk_skel is None:
                trunk_skel = trunk_path

        self.net_r = np.asarray(net_r, dtype=np.int64)
        self.net_c = np.asarray(net_c, dtype=np.int64)
        self.net_tan = np.asarray(net_tan, dtype=np.float32)
        self.net_factor = np.asarray(net_factor, dtype=np.float32)
        self.net_half = np.asarray(net_half, dtype=np.float32)
        print(f"  河网节点: {len(self.net_r):,}（主流 + 分流）")

        # 4. 主流中心线 → CSV（沿用原有平滑/重采样/UE 坐标逻辑）
        if trunk_skel is not None and len(trunk_skel) >= 2:
            self._write_centerline(trunk_skel, scale_factor, H, W, branch_segs)
        print(f"  河网提取耗时 {time.time()-t0:.1f}s")

    def _write_centerline(self, trunk_path, scale_factor, H, W, branch_segs):
        """将主流骨架路径平滑重采样后写 CSV，并输出河网预览图。"""
        from scipy.ndimage import gaussian_filter1d

        path_full = [(r / scale_factor, c / scale_factor) for r, c in trunk_path]
        path_arr = np.array(path_full, dtype=float)
        sigma = 3.0
        r_smooth = gaussian_filter1d(path_arr[:, 0], sigma=sigma)
        c_smooth = gaussian_filter1d(path_arr[:, 1], sigma=sigma)

        sample_spacing_cm = 500.0
        sample_spacing_px = sample_spacing_cm / UE_PER_PX10
        diffs = np.diff(np.stack([r_smooth, c_smooth], axis=1), axis=0)
        arc = np.cumsum(np.linalg.norm(diffs, axis=1))
        arc = np.insert(arc, 0, 0.0)
        total_arc = arc[-1]
        n_samples = max(2, int(np.ceil(total_arc / sample_spacing_px)) + 1)
        sample_arc = np.linspace(0.0, total_arc, n_samples)

        r_sampled = np.interp(sample_arc, arc, r_smooth)
        c_sampled = np.interp(sample_arc, arc, c_smooth)

        x = ue_x_of_col(c_sampled, W)
        y = ue_y_csv_of_row(r_sampled, H)
        r_i = np.clip(np.round(r_sampled).astype(np.int64), 0, H - 1)
        c_i = np.clip(np.round(c_sampled).astype(np.int64), 0, W - 1)
        z = self.dem_10x[r_i, c_i] * UE_SCALE

        points = [{"x": float(x[i]), "y": float(y[i]), "z": float(z[i])}
                  for i in range(len(x))]
        self.centerline_points = np.column_stack([x, y, z])

        out = Path(self.out_dir)
        out.mkdir(parents=True, exist_ok=True)
        csv_path = out / "terrain_water_centerline.csv"
        lines = ["---,Position"]
        for i, p in enumerate(points):
            pos = f"\"(X={p['x']:.6f},Y={p['y']:.6f},Z={p['z']:.6f})\""
            lines.append(f"Point{i},{pos}")
        csv_path.write_text("\n".join(lines), encoding="utf-8")
        print(f"  写入中心线 CSV: {csv_path}")

        # 预览：背景白 + 水体黑 + 主流红 + 分流按因子着色（避免蓝色水体与低因子蓝色混叠）
        preview = np.full((H, W, 3), 255, dtype=np.uint8)
        preview[self.water_mask] = [0, 0, 0]
        rr = np.clip(np.round(r_sampled).astype(np.int64), 0, H - 1)
        cc = np.clip(np.round(c_sampled).astype(np.int64), 0, W - 1)
        for i in range(len(rr) - 1):
            cv2.line(preview, (int(cc[i]), int(rr[i])),
                     (int(cc[i + 1]), int(rr[i + 1])), (0, 0, 255), thickness=10)

        inv = 1.0 / scale_factor
        for (r1, c1, r2, c2, fac) in branch_segs:
            color = self._factor_color(fac)
            cv2.line(preview,
                     (int(round(c1 * inv)), int(round(r1 * inv))),
                     (int(round(c2 * inv)), int(round(r2 * inv))),
                     (int(color[0]), int(color[1]), int(color[2])), thickness=5)
        preview_path = out / "terrain_water_centerline_preview.png"
        cv2.imwrite(str(preview_path), preview)
        print(f"  写入河网预览: {preview_path}")

    # ═══════════════════════════════════════════════════════════════
    # 静态速度场生成
    # ═══════════════════════════════════════════════════════════════

    def _generate_velocity_field(self, resolution: int = 2048):
        """基于河网（主流切向 + 分流衰减）生成水体速度场纹理。

        输出通道：
          R：世界空间流向 X 分量（[-1,1] 映射到 [0,1]）
          G：世界空间流向 Y 分量（[-1,1] 映射到 [0,1]）
          B：速度大小（中心快、岸边慢，分流越远越慢，叠加轻微噪声）
          A：水体掩膜

        纹理按 V-up 生成，PNG 顶部存图像底部，与 UE 里水体 mesh UV 方向一致。
        """
        from scipy.spatial import cKDTree

        if self.net_r is None or len(self.net_r) < 2:
            print("  河网节点缺失，跳过速度场生成")
            return

        print("生成水体速度场纹理...")
        t0 = time.time()
        res_h, res_w = self.water_small.shape
        water_idx = np.argwhere(self.water_small)  # (M, 2) [r, c]
        if len(water_idx) == 0:
            print("  无水体像素，跳过速度场生成")
            return

        net_pts = np.column_stack([self.net_r, self.net_c]).astype(np.float32)
        net_tree = cKDTree(net_pts)

        # 每个水体像素：最近河网节点 → 切向 + 因子 + 半宽
        dist_net, idx_net = net_tree.query(water_idx.astype(np.float32), k=1)
        tangent = self.net_tan[idx_net].copy()          # (M, 2)
        factor = self.net_factor[idx_net]               # (M,)
        half = np.maximum(self.net_half[idx_net], 0.5)  # (M,)
        channel_pos = np.clip(dist_net / half, 0.0, 1.0)  # 0=河道中心 1=岸边

        # 岸边切线场（高斯模糊 + Sobel，垂直梯度方向即切线）
        bd_idx = np.argwhere(~self.water_small)
        if len(bd_idx) == 0:
            print("  无岸边，跳过速度场生成")
            return
        bd_tree = cKDTree(bd_idx.astype(np.float32))
        blurred = cv2.GaussianBlur(self.water_small.astype(np.float32), (0, 0), sigmaX=8.0)
        gx = cv2.Sobel(blurred, cv2.CV_32F, 1, 0, ksize=5)
        gy = cv2.Sobel(blurred, cv2.CV_32F, 0, 1, ksize=5)
        grad_norm = np.sqrt(gx * gx + gy * gy)
        grad_norm[grad_norm < 1e-8] = 1.0
        tangent_r = -gx / grad_norm
        tangent_c = gy / grad_norm
        bd_tan = np.column_stack([
            tangent_r[bd_idx[:, 0], bd_idx[:, 1]],
            tangent_c[bd_idx[:, 0], bd_idx[:, 1]],
        ]).astype(np.float32)
        dist_bd, idx_bd = bd_tree.query(water_idx.astype(np.float32), k=1)
        bank_tangent = bd_tan[idx_bd]
        dot = np.sum(tangent * bank_tangent, axis=1)
        bank_tangent[dot < 0] *= -1

        # 岸边影响：近岸用岸边切向，河道中心用河网切向
        bank_influence = np.power(channel_pos, 1.5)
        direction = (1.0 - bank_influence[:, None]) * tangent + \
                    bank_influence[:, None] * bank_tangent

        # 低频 Perlin 噪声扰动方向
        noise = self._perlin_noise(res_h, res_w, octaves=4, base_grid=16, seed=self.velocity_seed)
        noise_vals = noise[water_idx[:, 0], water_idx[:, 1]]
        noise_angle = noise_vals * self.velocity_noise_scale
        cos_a = np.cos(noise_angle)
        sin_a = np.sin(noise_angle)
        rot_dir = np.empty_like(direction)
        rot_dir[:, 0] = direction[:, 0] * cos_a - direction[:, 1] * sin_a
        rot_dir[:, 1] = direction[:, 0] * sin_a + direction[:, 1] * cos_a
        direction = rot_dir

        dnorm = np.linalg.norm(direction, axis=1, keepdims=True)
        dnorm[dnorm < 1e-8] = 1.0
        direction = direction / dnorm

        # 向心偏转后处理：流线轻微弯向中心线（中心/岸边/分流远端均收敛）
        # 强度 s = k * 4p(1-p) * factor，方向取“指向中心”的垂直分量；
        # 主体仍沿流只是微弯，k=0 时此步完全等价于关闭。
        if self.velocity_converge > 0.0:
            pixel_xy = water_idx.astype(np.float32)
            to_center = net_pts[idx_net] - pixel_xy
            dnet_safe = np.maximum(dist_net, 1e-3)
            cvec = to_center / dnet_safe[:, None]          # 指向最近河网节点
            cpar = np.sum(cvec * direction, axis=1, keepdims=True)
            cperp = cvec - cpar * direction                # 垂直于流向的横向分量
            cn = np.linalg.norm(cperp, axis=1, keepdims=True)
            cn[cn < 1e-6] = 1.0
            cperp = cperp / cn
            s_bend = self.velocity_converge * 4.0 * channel_pos * (1.0 - channel_pos) * factor
            direction = direction + s_bend[:, None] * cperp
            dnorm2 = np.linalg.norm(direction, axis=1, keepdims=True)
            dnorm2[dnorm2 < 1e-8] = 1.0
            direction = direction / dnorm2

        # 速度 = 横向廓线(中心快岸边慢) × 分流衰减因子 × 噪声
        speed = (1.0 - channel_pos) * factor
        speed = speed * (1.0 + 0.2 * noise_vals)
        speed = np.clip(speed, 0.0, 1.0)

        # 组装输出纹理（V-up）
        tex = np.zeros((res_h, res_w, 4), dtype=np.float32)
        row_vup = res_h - 1 - water_idx[:, 0]
        tex[row_vup, water_idx[:, 1], 0] = direction[:, 1] * 0.5 + 0.5
        tex[row_vup, water_idx[:, 1], 1] = direction[:, 0] * 0.5 + 0.5
        tex[row_vup, water_idx[:, 1], 2] = speed
        tex[row_vup, water_idx[:, 1], 3] = 1.0

        out = Path(self.out_dir)
        out.mkdir(parents=True, exist_ok=True)
        vel_path = out / "terrain_velocity_field.png"
        vel_uint8 = np.clip(tex * 255.0, 0, 255).astype(np.uint8)
        cv2.imwrite(str(vel_path), cv2.cvtColor(vel_uint8, cv2.COLOR_RGBA2BGRA))
        print(f"  写入速度场纹理: {vel_path}")

        preview = self._velocity_preview(direction, speed, water_idx, res_h, res_w)
        preview_path = out / "terrain_velocity_field_preview.png"
        cv2.imwrite(str(preview_path), preview)
        print(f"  写入速度场预览: {preview_path}")

        print(f"  速度场生成耗时 {time.time()-t0:.1f}s")

    @staticmethod
    def _perlin_noise(h, w, octaves=4, base_grid=16, seed=42):
        """标准低频分形 Perlin 噪声，输出范围约 [-1, 1]。"""
        rng = np.random.default_rng(seed)

        def perlin_layer(gh, gw):
            angles = rng.random((gh, gw), dtype=np.float32) * 2.0 * np.pi
            grad_y, grad_x = np.cos(angles), np.sin(angles)

            xs = np.linspace(0.0, gw - 1.0, w, endpoint=False, dtype=np.float32)
            ys = np.linspace(0.0, gh - 1.0, h, endpoint=False, dtype=np.float32)
            xv, yv = np.meshgrid(xs, ys, indexing='xy')

            x0 = np.floor(xv).astype(np.int32)
            y0 = np.floor(yv).astype(np.int32)
            xf = xv - x0.astype(np.float32)
            yf = yv - y0.astype(np.float32)

            u = xf * xf * (3.0 - 2.0 * xf)
            v = yf * yf * (3.0 - 2.0 * yf)

            x1 = np.clip(x0 + 1, 0, gw - 1)
            y1 = np.clip(y0 + 1, 0, gh - 1)
            x0 = np.clip(x0, 0, gw - 1)
            y0 = np.clip(y0, 0, gh - 1)

            n00 = grad_x[y0, x0] * xf + grad_y[y0, x0] * yf
            n01 = grad_x[y0, x1] * (xf - 1.0) + grad_y[y0, x1] * yf
            n10 = grad_x[y1, x0] * xf + grad_y[y1, x0] * (yf - 1.0)
            n11 = grad_x[y1, x1] * (xf - 1.0) + grad_y[y1, x1] * (yf - 1.0)

            nx0 = n00 + (n01 - n00) * u
            nx1 = n10 + (n11 - n10) * u
            return nx0 + (nx1 - nx0) * v

        noise = np.zeros((h, w), dtype=np.float32)
        freq, amp, total = 1.0, 1.0, 0.0
        for _ in range(octaves):
            gh = max(2, int(round(base_grid * freq)))
            gw = max(2, int(round(base_grid * freq)))
            noise += perlin_layer(gh, gw) * amp
            total += amp
            freq *= 2.0
            amp *= 0.5

        return noise / total

    @staticmethod
    def _velocity_preview(direction, profile, water_idx, h, w):
        """HSV 流向预览：色相=方向，亮度=速度（V-up 布局，与 UE 纹理一致）。"""
        angle = np.arctan2(direction[:, 0], direction[:, 1])
        hue = ((angle + np.pi) / (2.0 * np.pi) * 180).astype(np.uint8)
        val = (profile * 255).astype(np.uint8)
        hsv = np.zeros((h, w, 3), dtype=np.uint8)
        row_vup = h - 1 - water_idx[:, 0]
        hsv[row_vup, water_idx[:, 1], 0] = hue
        hsv[row_vup, water_idx[:, 1], 1] = 255
        hsv[row_vup, water_idx[:, 1], 2] = val
        return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

    # ═══════════════════════════════════════════════════════════════
    # 主流程
    # ═══════════════════════════════════════════════════════════════

    def build(self):
        """提取河网并生成速度场纹理。"""
        t0 = time.time()
        self.load()
        self._extract_network()
        if self.net_r is not None and len(self.net_r) >= 2:
            self._generate_velocity_field(resolution=self.velocity_res)
        print(f"总耗时: {time.time()-t0:.1f}s")


# ═══════════════════════════════════════════════════════════════
# 独立测试入口 / CLI
# ═══════════════════════════════════════════════════════════════

def test(label_10x_path: str = None, dem_path: str = None, out_dir: str = None):
    """检测流场生成器输入是否有效，无效则报错，有效则运行。"""
    builder = WaterFlowBuilder(
        label_10x_path=label_10x_path,
        dem_path=dem_path,
        out_dir=out_dir,
    )
    for pth in (builder.label_10x_path, builder.dem_path):
        if not Path(pth).is_file():
            raise FileNotFoundError(f"缺少输入: {pth}")
    builder.build()


def main():
    p = argparse.ArgumentParser(description="水体河网 + 分流速度场生成器")
    p.add_argument("--label-10x", default="TestInput/SanHe/Output_10x/labels_10x_no_boundary.tif",
                   help="10x 语义标签路径（水体 = band1 == 0）")
    p.add_argument("--dem", default="TestInput/SanHe/dem.tif", help="DEM 路径")
    p.add_argument("--out-dir", default="output/terrain_adaptive", help="输出目录")
    p.add_argument("--velocity-res", type=int, default=2048, help="速度场纹理最长边像素数")
    p.add_argument("--velocity-seed", type=int, default=42, help="Perlin 噪声随机种子")
    p.add_argument("--velocity-noise", type=float, default=0.15, help="流向噪声强度（弧度）")
    p.add_argument("--velocity-branch-alpha", type=float, default=0.5,
                   help="分流速度衰减幂次 α，factor = (1 - d/L)^α")
    p.add_argument("--velocity-branch-residual", type=float, default=0.1,
                   help="分流末端残余速度因子（0~1）")
    p.add_argument("--velocity-converge", type=float, default=0.0,
                   help="向心偏转强度 k（0=关闭；流线轻微弯向中心线，中心/岸边/分流远端收敛）")
    args = p.parse_args()
    builder = WaterFlowBuilder(
        label_10x_path=args.label_10x,
        dem_path=args.dem,
        out_dir=args.out_dir,
        velocity_res=args.velocity_res,
        velocity_seed=args.velocity_seed,
        velocity_noise_scale=args.velocity_noise,
        velocity_branch_alpha=args.velocity_branch_alpha,
        velocity_branch_residual=args.velocity_branch_residual,
        velocity_converge=args.velocity_converge,
    )
    for pth in (builder.label_10x_path, builder.dem_path):
        if not Path(pth).is_file():
            raise FileNotFoundError(f"缺少输入: {pth}")
    builder.build()


if __name__ == "__main__":
    main()

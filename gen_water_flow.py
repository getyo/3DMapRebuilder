#!/usr/bin/env python3
"""
水体流场生成器（中心线 + 静态速度场）— 独立模块

输入：
  - labels_10x_no_boundary.tif   10x 语义标签（水体 = band1 == 0）
  - dem.tif                      高程（1x 分辨率，内部升采样到 10x）

输出（写入 out_dir）：
  - terrain_water_centerline.csv              河流中心线（UE DataTable 格式）
  - terrain_water_centerline_preview.png      中心线预览图
  - terrain_velocity_field.png                静态速度场纹理（RGBA）
  - terrain_velocity_field_preview.png        速度场 HSV 预览图

依赖关系：
  水体掩膜必须来自 label_postprocess 平滑后的 10x 标签，保证与
  terrain_water.obj 的水体区域一致；本模块不依赖 Delaunay / 地形网格，
  可与地形 OBJ、3DGS 语义地图并行生成（主流程见
  gen_adaptive_terrain_centerline.py 的三路并行编排）。

水体策略（与原地形模块行为保持一致）：
  - 中心线：对水体掩膜下采样后骨架化，取最大连通分量的直径路径，
    高斯平滑后按弧长重采样输出。多段不相连水体仍以最大连通分量驱动
    整片水体的速度场，暂不区分河 / 湖。

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
from map_common import (UE_SCALE, UE_PER_PX10, PX10_PER_UE,
                        ue_x_of_col, ue_y_csv_of_row)


class WaterFlowBuilder:
    """
    从 10x 水体掩膜 + DEM 提取河流中心线并生成静态速度场纹理。

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
    ):
        self.label_10x_path = label_10x_path
        self.dem_path = dem_path
        self.out_dir = out_dir
        # 速度场参数
        self.velocity_res = velocity_res          # 纹理最长边像素数
        self.velocity_seed = velocity_seed        # Perlin 噪声随机种子
        self.velocity_noise_scale = velocity_noise_scale  # 流向噪声强度（弧度）

        self.H10 = self.W10 = 0
        self.water_mask = None
        self.dem_10x = None
        self.centerline_points = None   # 中心线世界坐标（N, 3）：x=east, y=+south, z=height

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
        sample_spacing_px = sample_spacing_cm / UE_PER_PX10
        diffs = np.diff(np.stack([r_smooth, c_smooth], axis=1), axis=0)
        arc = np.cumsum(np.linalg.norm(diffs, axis=1))
        arc = np.insert(arc, 0, 0.0)
        total_arc = arc[-1]
        n_samples = max(2, int(np.ceil(total_arc / sample_spacing_px)) + 1)
        sample_arc = np.linspace(0.0, total_arc, n_samples)

        r_sampled = np.interp(sample_arc, arc, r_smooth)
        c_sampled = np.interp(sample_arc, arc, c_smooth)

        # 6. 像素坐标 → UE 世界坐标
        # 注意：OBJ 在 UE 导入时会做右手系→左手系转换，Y 被翻转；
        # CSV 直接作为 UE 世界坐标读入，因此用 ue_y_csv_of_row（= -(OBJ 内 y)），
        # 与 UE 导入 terrain_water.obj 后的 mesh Y 轴（y=+south）保持一致。
        x = ue_x_of_col(c_sampled, W)
        y = ue_y_csv_of_row(r_sampled, H)

        r_i = np.clip(np.round(r_sampled).astype(np.int64), 0, H - 1)
        c_i = np.clip(np.round(c_sampled).astype(np.int64), 0, W - 1)
        z = self.dem_10x[r_i, c_i] * UE_SCALE

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

        # 8. 输出预览图
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

    # ═══════════════════════════════════════════════════════════════
    # 静态速度场生成
    # ═══════════════════════════════════════════════════════════════

    def _generate_velocity_field(self, resolution: int = 2048):
        """以中心线为主、岸边切线影响、加低频扰动生成水体速度场纹理。

        输出通道：
          R：世界空间流向 X 分量（[-1,1] 映射到 [0,1]）
          G：世界空间流向 Y 分量（[-1,1] 映射到 [0,1]）
          B：速度大小（中心≈1，岸边≈0，叠加轻微噪声）
          A：水体掩膜

        纹理按 V-up 生成，PNG 顶部存图像底部，与 UE 里水体 mesh UV 方向一致。
        """
        if self.centerline_points is None or len(self.centerline_points) < 2:
            print("  中心线缺失，跳过速度场生成")
            return

        print("生成水体速度场纹理...")
        t0 = time.time()
        H, W = self.water_mask.shape

        # 1. 计算速度场分辨率
        max_side = max(W, H)
        if max_side <= resolution:
            res_w, res_h = W, H
        else:
            scale = resolution / max_side
            res_w = int(round(W * scale))
            res_h = int(round(H * scale))
        print(f"  速度场分辨率: {res_w}x{res_h}")

        # 2. 水体像素坐标（下采样到目标分辨率）
        if res_h != H or res_w != W:
            water_mask_small = cv2.resize(
                self.water_mask.astype(np.uint8) * 255,
                (res_w, res_h),
                interpolation=cv2.INTER_NEAREST,
            ) > 0
        else:
            water_mask_small = self.water_mask
        water_idx = np.argwhere(water_mask_small)  # (M, 2) [r, c]
        if len(water_idx) == 0:
            print("  无水体像素，跳过速度场生成")
            return

        # 3. 中心线切线（在速度场分辨率下）
        cl = self.centerline_points[:, :2].astype(np.float64)
        c_cl = cl[:, 0] * PX10_PER_UE + W * 0.5
        r_cl = cl[:, 1] * PX10_PER_UE + H * 0.5
        c_cl *= res_w / W
        r_cl *= res_h / H
        cl_res = np.column_stack([r_cl, c_cl]).astype(np.float32)

        tangents = np.empty_like(cl_res)
        tangents[1:-1] = cl_res[2:] - cl_res[:-2]
        tangents[0] = cl_res[1] - cl_res[0]
        tangents[-1] = cl_res[-1] - cl_res[-2]
        norms = np.linalg.norm(tangents, axis=1, keepdims=True)
        norms[norms < 1e-8] = 1.0
        tangents = tangents / norms

        from scipy.spatial import cKDTree
        cl_tree = cKDTree(cl_res)

        # 4. 平滑岸边切线场（高斯模糊 + Sobel 梯度，垂直方向即为切线）
        bd_mask_small = ~water_mask_small
        bd_idx = np.argwhere(bd_mask_small)
        if len(bd_idx) == 0:
            print("  无岸边，跳过速度场生成")
            return
        bd_tree = cKDTree(bd_idx.astype(np.float32))

        blurred = cv2.GaussianBlur(water_mask_small.astype(np.float32), (0, 0), sigmaX=8.0)
        gx = cv2.Sobel(blurred, cv2.CV_32F, 1, 0, ksize=5)
        gy = cv2.Sobel(blurred, cv2.CV_32F, 0, 1, ksize=5)
        grad_norm = np.sqrt(gx * gx + gy * gy)
        grad_norm[grad_norm < 1e-8] = 1.0
        # 切线场 (r, c) 方向 = 垂直于梯度
        tangent_r = -gx / grad_norm
        tangent_c = gy / grad_norm
        bd_tangents = np.column_stack([
            tangent_r[bd_idx[:, 0], bd_idx[:, 1]],
            tangent_c[bd_idx[:, 0], bd_idx[:, 1]],
        ]).astype(np.float32)

        # 5. 每个水体像素：中心线切线 + 岸边切线加权
        dist_cl, idx_cl = cl_tree.query(water_idx.astype(np.float32), k=1)
        direction = tangents[idx_cl].copy()

        dist_bd, idx_bd = bd_tree.query(water_idx.astype(np.float32), k=1)
        bank_tangent = bd_tangents[idx_bd]
        # 岸边切线可能与中心线反向，统一方向
        dot = np.sum(direction * bank_tangent, axis=1)
        bank_tangent[dot < 0] *= -1

        # 岸边影响：离岸边越近影响越大，中心线处影响为 0
        max_bank_dist = max(dist_bd.max(), 1e-6)
        bank_influence = 1.0 - np.clip(dist_bd / max_bank_dist, 0.0, 1.0)
        bank_influence = np.power(bank_influence, 1.5)

        direction = (1.0 - bank_influence[:, None]) * direction + \
                    bank_influence[:, None] * bank_tangent

        # 6. 低频 Perlin 噪声扰动方向与速度
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

        # 7. 速度：中心快、岸边慢，叠加轻微噪声
        speed = np.clip(dist_bd / max_bank_dist, 0.0, 1.0)
        speed = speed * (1.0 + 0.2 * noise_vals)
        speed = np.clip(speed, 0.0, 1.0)

        # 8. 组装输出纹理（V-up）
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
        """提取中心线并生成速度场纹理。"""
        t0 = time.time()
        self.load()
        self._extract_centerline()
        if self.centerline_points is not None:
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
    p = argparse.ArgumentParser(description="水体中心线 + 静态速度场生成器")
    p.add_argument("--label-10x", default="TestInput/SanHe/Output_10x/labels_10x_no_boundary.tif",
                   help="10x 语义标签路径（水体 = band1 == 0）")
    p.add_argument("--dem", default="TestInput/SanHe/dem.tif", help="DEM 路径")
    p.add_argument("--out-dir", default="output/terrain_adaptive", help="输出目录")
    p.add_argument("--velocity-res", type=int, default=2048, help="速度场纹理最长边像素数")
    p.add_argument("--velocity-seed", type=int, default=42, help="Perlin 噪声随机种子")
    p.add_argument("--velocity-noise", type=float, default=0.15, help="流向噪声强度（弧度）")
    args = p.parse_args()
    test(args.label_10x, args.dem, args.out_dir)


if __name__ == "__main__":
    main()

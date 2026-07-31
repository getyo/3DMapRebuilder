#!/usr/bin/env python3
"""
语义标签后处理器（10x 上采样 + 轮廓平滑）

输入: 分类器输出的 labels.tif（三通道语义标签）
输出:
  labels_10x_no_boundary.tif  — 上采样后的语义标签图
  boundary_lines_10x.png      — 独立轮廓线图（RGBA）
  preview_classification_noboundary.png — 可视化预览
"""

import os
import cv2
import numpy as np
import scipy.ndimage

try:
    import rasterio
    HAS_RASTERIO = True
except ImportError:
    HAS_RASTERIO = False

from classify_vecw import VecClassifier, SANHE_VEC, SANHE_SATE, SANHE_LABEL_DIR
from gen_semantic import SemanticMapBuilder


# ═══════════════════════════════════════════════════════════════
# 标签后处理器（非单例）
# ═══════════════════════════════════════════════════════════════

class LabelPostprocessor:
    """
    从分类器结果 labels.tif 生成 10x 上采样标签与轮廓图。

    用法:
      pp = LabelPostprocessor(input_path='...', output_dir='...')
      pp.process()

    或链式设置:
      pp = LabelPostprocessor()
      pp.set_input(classifier.label_out).set_output('...').process()
    """

    # 默认路径与分类器输出保持一致
    DEFAULT_INPUT  = "TestInput/SanHe/labels.tif"
    DEFAULT_OUTPUT = "TestInput/SanHe/Output_10x"
    SCALE          = 10

    def __init__(self, input_path: str = None, output_dir: str = None):
        self.input_path = input_path or self.DEFAULT_INPUT
        self.output_dir = output_dir or self.DEFAULT_OUTPUT

        # 运行时数据
        self._profile    = None
        self._band1      = None
        self._band2      = None
        self._band3      = None
        self._band1_10x  = None
        self._band2_10x  = None
        self._out_band1  = None
        self._out_band2  = None
        self._out_band3  = None
        self._boundary   = None
        self._H          = 0
        self._W          = 0
        self._H_10x      = 0
        self._W_10x      = 0

    # ── 输入输出设置 ──
    def set_input(self, input_path: str):
        """设置输入 labels.tif 路径（分类器结果）"""
        self.input_path = input_path
        return self

    def set_output(self, output_dir: str):
        """设置输出目录"""
        self.output_dir = output_dir
        return self

    # ═══════════════════════════════════════════════════════════════
    # 主流程
    # ═══════════════════════════════════════════════════════════════

    def process(self):
        """执行完整后处理流程"""
        os.makedirs(self.output_dir, exist_ok=True)

        self._load_input()
        self._upsample()
        self._init_output_arrays()
        self._process_classes()
        self._propagate_road_levels()
        self._build_height_band()
        self._visualize_and_save()

        print("处理完成！拐角位置已被 100% 冻结防膨胀，段内保持平直与流畅。")

    # ═══════════════════════════════════════════════════════════════
    # 内部流程
    # ═══════════════════════════════════════════════════════════════

    def _load_input(self):
        """读取分类器输出的三通道标签图"""
        print(f"正在读取文件: {self.input_path}")
        if HAS_RASTERIO and self.input_path.endswith(('.tif', '.tiff')):
            with rasterio.open(self.input_path) as src:
                labels = src.read()
                self._profile = src.profile.copy()
            self._band1 = labels[0]  # class_id
            self._band2 = labels[1]  # road_level
            self._band3 = labels[2]  # height
        else:
            img = cv2.imread(self.input_path, cv2.IMREAD_UNCHANGED)
            if len(img.shape) == 3 and img.shape[2] >= 3:
                self._band1 = img[:, :, 0]
                self._band2 = img[:, :, 1] if img.shape[2] > 1 else np.zeros_like(self._band1)
                self._band3 = img[:, :, 2] if img.shape[2] > 2 else np.zeros_like(self._band1)
            else:
                self._band1 = img
                self._band2 = np.zeros_like(self._band1)
                self._band3 = np.zeros_like(self._band1)
            self._profile = None

        self._H, self._W = self._band1.shape
        self._H_10x = self._H * self.SCALE
        self._W_10x = self._W * self.SCALE
        print(f"原始分辨率: {self._W}x{self._H}  --->  10倍超采样分辨率: {self._W_10x}x{self._H_10x}")

    def _upsample(self):
        """10倍最近邻上采样"""
        self._band1_10x = cv2.resize(
            self._band1, (self._W_10x, self._H_10x), interpolation=cv2.INTER_NEAREST
        )
        self._band2_10x = cv2.resize(
            self._band2, (self._W_10x, self._H_10x), interpolation=cv2.INTER_NEAREST
        )

    def _init_output_arrays(self):
        """初始化输出数组"""
        self._out_band1 = np.full((self._H_10x, self._W_10x), 60, dtype=np.uint8)
        self._boundary  = np.zeros((self._H_10x, self._W_10x, 4), dtype=np.uint8)

    def _process_classes(self):
        """分类别独立平滑处理"""
        classes_config = [
            (0,  "water",    20.0, 500, (239, 198, 171, 255)),  # 水体: 高斯平滑
            (40, "building", 15.0, 300, (243, 250, 249, 255)),  # 建筑: 高斯平滑
            (20, "road",     12.0, 150, (120, 205, 254, 255)),  # 道路: 冻结拐角 + 段内插值
        ]

        for class_id, mode, param, min_area, line_color in classes_config:
            print(f"正在处理 Class {class_id} ({mode})...")
            mask = (self._band1_10x == class_id).astype(np.uint8) * 255
            if not np.any(mask):
                continue

            # 道路不做大核形态学闭运算，防止提前填平路口
            kernel_size = (3, 3) if mode == "road" else (11, 11)
            kernel_shape = cv2.MORPH_RECT if mode == "road" else cv2.MORPH_ELLIPSE
            kernel = cv2.getStructuringElement(kernel_shape, kernel_size)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

            for cnt in contours:
                if cv2.contourArea(cnt) < min_area:
                    continue

                if mode == "road":
                    # 道路专属：冻结拐角 + 段内插值
                    smooth_cnt = self._smooth_road_freeze_corners(
                        cnt, epsilon=param, corner_angle_thresh=150.0, chaikin_iters=2
                    )
                else:
                    smooth_cnt = self._smooth_contour_gaussian(cnt, sigma=param)

                # (A) 绘制到无描边分类图
                cv2.drawContours(self._out_band1, [smooth_cnt], -1, class_id, thickness=-1)

                # (B) 绘制到独立边界线图
                cv2.drawContours(self._boundary, [smooth_cnt], -1, line_color, thickness=3)

    def _propagate_road_levels(self):
        """精准继承 Band 2，消灭杂色与淡绿边"""
        self._out_band2 = np.zeros((self._H_10x, self._W_10x), dtype=np.uint8)
        road_idx = (self._out_band1 == 20)
        valid_road_src = (self._band1_10x == 20) & (self._band2_10x > 0)

        if np.any(valid_road_src) and np.any(road_idx):
            _, indices = scipy.ndimage.distance_transform_edt(~valid_road_src, return_indices=True)
            propagated_band2 = self._band2_10x[indices[0], indices[1]]
            self._out_band2[road_idx] = propagated_band2[road_idx]

    def _build_height_band(self):
        """构建 Band 3 高度通道"""
        self._out_band3 = np.zeros((self._H_10x, self._W_10x), dtype=np.uint8)
        self._out_band3[self._out_band1 == 40] = 120  # 建筑高度

    def _visualize_and_save(self):
        """可视化预览并保存所有输出"""
        print("正在生成分类预览图...")
        vis_rgb = np.zeros((self._H_10x, self._W_10x, 3), dtype=np.uint8)
        vis_rgb[self._out_band1 == 60] = [30, 0, 50]       # 地面: 深红棕
        vis_rgb[self._out_band1 == 40] = [120, 10, 80]     # 建筑: 紫蓝色
        vis_rgb[self._out_band1 == 0]  = [0, 0, 0]         # 水体: 黑色

        road_pixels = np.where(self._out_band1 == 20)
        for r, c in zip(road_pixels[0], road_pixels[1]):
            lvl = self._out_band2[r, c]
            if lvl == 40:      # 高速
                vis_rgb[r, c] = [241, 160, 186]
            elif lvl == 80:    # 国道
                vis_rgb[r, c] = [120, 205, 254]
            elif lvl == 120:   # 省道
                vis_rgb[r, c] = [130, 235, 254]
            else:              # 支路/普通路
                vis_rgb[r, c] = [0, 230, 50]

        out_labels = np.stack([self._out_band1, self._out_band2, self._out_band3], axis=0)

        path_labels_tif = os.path.join(self.output_dir, "labels_10x_no_boundary.tif")
        path_boundary_png = os.path.join(self.output_dir, "boundary_lines_10x.png")
        path_vis_png = os.path.join(self.output_dir, "preview_classification_noboundary.png")

        cv2.imwrite(path_boundary_png, self._boundary)
        cv2.imwrite(path_vis_png, cv2.cvtColor(vis_rgb, cv2.COLOR_RGB2BGR))

        if HAS_RASTERIO and self._profile is not None:
            self._profile.update(
                dtype=rasterio.uint8,
                count=3,
                width=self._W_10x,
                height=self._H_10x,
                transform=self._profile['transform'] * self._profile['transform'].scale(0.1, 0.1)
            )
            with rasterio.open(path_labels_tif, 'w', **self._profile) as dst:
                dst.write(out_labels)
            print(f"[OK] 10x GeoTIFF 已保存: {path_labels_tif}")

    # ═══════════════════════════════════════════════════════════════
    # 轮廓平滑工具（内部静态方法）
    # ═══════════════════════════════════════════════════════════════

    @staticmethod
    def _smooth_contour_gaussian(cnt, sigma=15.0):
        """通用高斯平滑（用于水体和建筑）"""
        pts = cnt.reshape(-1, 2).astype(float)
        if len(pts) < 6:
            return cnt
        x_smooth = scipy.ndimage.gaussian_filter1d(pts[:, 0], sigma=sigma, mode='wrap')
        y_smooth = scipy.ndimage.gaussian_filter1d(pts[:, 1], sigma=sigma, mode='wrap')
        smoothed = np.stack([x_smooth, y_smooth], axis=1).astype(np.int32)
        return smoothed.reshape(-1, 1, 2)

    @staticmethod
    def _smooth_road_freeze_corners(cnt, epsilon=12.0, corner_angle_thresh=150.0, chaikin_iters=2):
        """
        【先冻结拐角、后段内插值】专有道路平滑算法：
        1. 识别并锁定夹角 < 150 度的关键拐角（T字/十字路口内角），坐标 100% 冻结不动。
        2. 直线段 (A->B) 保持绝对直线；弯道段在端点冻结的前提下做段内 Chaikin 插值。
        """
        if len(cnt) < 4:
            return cnt.reshape(-1, 1, 2)

        # 1. 用 approxPolyDP 提纯主要骨架点（消除 10x 放大带来的像素阶梯噪点）
        approx = cv2.approxPolyDP(cnt, epsilon, True).reshape(-1, 2)
        m = len(approx)

        if m < 3:
            return cnt.reshape(-1, 1, 2)

        # 2. 计算每个顶点的夹角，检测并【冻结】关键拐角
        is_frozen = np.zeros(m, dtype=bool)
        for i in range(m):
            p_prev = approx[i - 1]
            p_curr = approx[i]
            p_next = approx[(i + 1) % m]

            v1 = p_prev - p_curr
            v2 = p_next - p_curr
            norm1 = np.linalg.norm(v1)
            norm2 = np.linalg.norm(v2)

            if norm1 > 1e-3 and norm2 > 1e-3:
                cos_angle = np.clip(np.dot(v1, v2) / (norm1 * norm2), -1.0, 1.0)
                angle_deg = np.degrees(np.arccos(cos_angle))
                # 夹角小于 150 度的关键拐角点，强行标记为冻结锚点
                if angle_deg < corner_angle_thresh:
                    is_frozen[i] = True

        frozen_indices = np.where(is_frozen)[0]

        # 如果没有检测到拐角（环形小路），退化为常规全封闭 Chaikin
        if len(frozen_indices) == 0:
            pts = approx.astype(float)
            for _ in range(chaikin_iters):
                new_pts = []
                n = len(pts)
                for i in range(n):
                    p0 = pts[i]
                    p1 = pts[(i + 1) % n]
                    new_pts.append(0.75 * p0 + 0.25 * p1)
                    new_pts.append(0.25 * p0 + 0.75 * p1)
                pts = np.array(new_pts)
            return pts.astype(np.int32).reshape(-1, 1, 2)

        # 3. 按冻结拐角拆分，进行端点固定（Fixed-Endpoint）的段内插值
        final_pts = []
        num_frozen = len(frozen_indices)

        for idx in range(num_frozen):
            i_start = frozen_indices[idx]
            i_end = frozen_indices[(idx + 1) % num_frozen]

            if i_end > i_start:
                seg = approx[i_start : i_end + 1]
            else:
                seg = np.vstack([approx[i_start:], approx[: i_end + 1]])

            # -- 段内插值/平滑 --
            if len(seg) == 2:
                # 只有首尾两个冻结拐角：绝对直线段！不需要任何切削插值
                smoothed_seg = seg.astype(float)
            else:
                # 有中间弯道点：保持首尾 Frozen 锚点 100% 锁死，仅对中间点做 Chaikin 插值
                pts_sub = seg.astype(float)
                for _ in range(chaikin_iters):
                    sub_new = [pts_sub[0]]  # 强行锁定起点 (Frozen Corner)
                    n_sub = len(pts_sub)
                    for k in range(n_sub - 1):
                        p0 = pts_sub[k]
                        p1 = pts_sub[k + 1]
                        q = 0.75 * p0 + 0.25 * p1
                        r = 0.25 * p0 + 0.75 * p1
                        if k > 0:
                            sub_new.append(q)
                        sub_new.append(r)
                    sub_new.append(pts_sub[-1])  # 强行锁定终点 (Frozen Corner)
                    pts_sub = np.array(sub_new)
                smoothed_seg = pts_sub

            # 组合各段（避免重复拼接首尾点）
            if len(final_pts) == 0:
                final_pts.extend(smoothed_seg)
            else:
                final_pts.extend(smoothed_seg[1:])

        final_arr = np.array(final_pts).astype(np.int32)
        return final_arr.reshape(-1, 1, 2)


# ═══════════════════════════════════════════════════════════════
# 入口
# ═══════════════════════════════════════════════════════════════

def main():
    """完整流程入口：分类 → 语义地图 → 标签后处理"""
    # 1. 语义分类
    classifier = VecClassifier()
    classifier.set_input(SANHE_VEC, SANHE_SATE)
    classifier.set_output(SANHE_LABEL_DIR)
    classifier.run()

    # 2. 语义地图生成
    builder = SemanticMapBuilder(label_path=classifier.label_out)
    builder.build()

    # 3. 标签后处理（输入与分类器输出保持一致）
    postprocessor = LabelPostprocessor(
        input_path=classifier.label_out,
        output_dir=os.path.join(SANHE_LABEL_DIR, "Output_10x")
    )
    postprocessor.process()


if __name__ == "__main__":
    main()

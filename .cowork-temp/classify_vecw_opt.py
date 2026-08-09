#!/usr/bin/env python3
"""
天地图 vec_w 语义标签分类器

输入: vec_w 拼接图 (RGBA PNG)
输出:
  water.tif     — 水体二值掩膜
  building.tif  — 建筑二值掩膜
  road.tif      — 道路二值掩膜
  labels.tif    — 三通道语义标签图
"""

import argparse
import math
from pathlib import Path
from typing import NamedTuple

import numpy as np
from PIL import Image
import rasterio
from scipy.ndimage import binary_closing


# ============ RGBA 颜色常量 ============
class RGBA(NamedTuple):
    """不可变 RGBA 颜色值"""
    R: int
    G: int
    B: int
    A: int = 255


# 精确标签颜色（基于用户提供的纯色样本）
COLOR_WATER    = RGBA(171, 198, 239)  # 水体
COLOR_BUILDING = RGBA(249, 250, 243)  # 建筑（比地面亮）
COLOR_GROUND   = RGBA(245, 244, 238)  # 地面

# 道路颜色（待用户填入主路、支路值）
COLOR_MAIN_ROAD   = RGBA(186, 160, 241)  # 高速  G=40
COLOR_BRANCH_ROAD = RGBA(254, 205, 120)  # 国道  G=80
COLOR_SMALL_ROAD  = RGBA(254, 235, 130)  # 省道  G=120
COLOR_BRANCH_PATH = RGBA(255, 255, 255)  # 支路  G=160
COLOR_PATH        = RGBA(253, 253, 253)  # 小路  G=200

# RGB容差
RGB_DIFF = 0


def _match_rgba(r, g, b, color: RGBA, opaque):
    """像素 (R,G,B) 是否匹配给定颜色(±1 容差)，且不透明"""
    return opaque & \
        (r >= color.R - RGB_DIFF) & (r <= color.R + RGB_DIFF) & \
        (g >= color.G - RGB_DIFF) & (g <= color.G + RGB_DIFF) & \
        (b >= color.B - RGB_DIFF) & (b <= color.B + RGB_DIFF)


# ============ 分类器 ============
class VecClassifier:
    """Vec 瓦片语义分类器（普通类，非单例）"""

    def __init__(self):
        # 默认空路径
        self.InputVec      = ""
        self.water_out     = ""
        self.building_out  = ""
        self.road_out      = ""
        self.label_out     = ""
        # 卫星图参考路径
        self.reference_sat = ""

    def set_input(self, vec_path: str, satellite_path: str):
        """设置输入 vec 拼接图路径"""
        self.InputVec = vec_path
        self.reference_sat = satellite_path

    def set_output(self, dir_path: str, prefix: str = ""):
        """设置输出目录和文件名前缀"""
        d = Path(dir_path)
        self.water_out    = str(d / f"{prefix}water.tif")
        self.building_out = str(d / f"{prefix}building.tif")
        self.road_out     = str(d / f"{prefix}road.tif")
        self.label_out    = str(d / f"{prefix}labels.tif")

    def _check_paths(self):
        """检查所有输入输出路径是否合法"""
        # 输入
        if not self.InputVec:
            raise RuntimeError("InputVec not set. Call set_input() first.")
        if not Path(self.InputVec).is_file():
            raise FileNotFoundError(f"Input vec image not found: {self.InputVec}")

        # 卫星图参考（用于读取 CRS/transform）
        if not self.reference_sat:
            raise RuntimeError("Reference satellite path is empty.")
        if not Path(self.reference_sat).is_file():
            raise FileNotFoundError(f"Reference satellite not found: {self.reference_sat}")

        # 输出
        if not self.water_out:
            raise RuntimeError("Output paths not set. Call set_output() first.")
        for name, path in [
            ("water",    self.water_out),
            ("building", self.building_out),
            ("road",     self.road_out),
            ("label",    self.label_out),
        ]:
            if not path:
                raise RuntimeError(f"{name}_out is empty.")
            out_dir = Path(path).parent
            if out_dir and not out_dir.is_dir():
                raise FileNotFoundError(f"Output directory does not exist: {out_dir}")

    # ──── 分类逻辑 ────

    def _classify_water(self, r, g, b, opaque):
        """水体 → COLOR_WATER ±1"""
        return _match_rgba(r, g, b, COLOR_WATER, opaque)

    def _classify_road(self, r, g, b, opaque, exclude_mask):
        """
        道路五级分类（±1 容差）
          - 高速 → COLOR_MAIN_ROAD    G=40
          - 国道 → COLOR_BRANCH_ROAD  G=80
          - 省道 → COLOR_SMALL_ROAD   G=120
          - 支路 → COLOR_BRANCH_PATH  G=160
          - 小路 → COLOR_PATH         G=200
        """
        lvl = np.zeros_like(r, dtype=np.uint8)

        road_highway  = _match_rgba(r, g, b, COLOR_MAIN_ROAD,   opaque) & ~exclude_mask
        road_national = _match_rgba(r, g, b, COLOR_BRANCH_ROAD, opaque) & ~exclude_mask & ~road_highway
        road_province = _match_rgba(r, g, b, COLOR_SMALL_ROAD,  opaque) & ~exclude_mask & ~road_highway & ~road_national
        road_branch   = _match_rgba(r, g, b, COLOR_BRANCH_PATH, opaque) & ~exclude_mask & ~road_highway & ~road_national & ~road_province
        road_path     = _match_rgba(r, g, b, COLOR_PATH,        opaque) & ~exclude_mask & ~road_highway & ~road_national & ~road_province & ~road_branch

        lvl[road_highway]  = 40
        lvl[road_national] = 80
        lvl[road_province] = 120
        lvl[road_branch]   = 160
        lvl[road_path]     = 200

        road_all = road_highway | road_national | road_province | road_branch | road_path
        return road_all, lvl

    def _classify_building(self, r, g, b, opaque, water_mask, exclude_mask):
        """建筑 → COLOR_BUILDING ±1 + 形态学闭合"""
        raw = _match_rgba(r, g, b, COLOR_BUILDING, opaque) & ~water_mask & ~exclude_mask
        se = np.ones((3, 3), dtype=bool)
        return binary_closing(raw, structure=se, iterations=1)

    def _classify_ground(self, r, g, b, opaque):
        """地面 → COLOR_GROUND ±1"""
        return _match_rgba(r, g, b, COLOR_GROUND, opaque)

    def run(self):
        """执行完整分类流程"""
        self._check_paths()

        # ── 加载 ──
        print("Loading vec_w mosaic...")
        img = Image.open(self.InputVec).convert("RGBA")
        pixels = np.array(img, dtype=np.uint8)
        H, W = pixels.shape[:2]
        print(f"  Size: {W}x{H}")

        r = pixels[:,:,0].astype(np.int32)
        g = pixels[:,:,1].astype(np.int32)
        b = pixels[:,:,2].astype(np.int32)
        a = pixels[:,:,3]

        opaque = a > 0

        # ── 初始化输出数组 ──
        class_id = np.full((H, W), 60, dtype=np.uint8)  # 默认地面(60)
        road_lvl = np.zeros((H, W), dtype=np.uint8)
        height   = np.zeros((H, W), dtype=np.uint8)

        # ── 1. 水体 ──
        water = self._classify_water(r, g, b, opaque)
        class_id[water] = 0
        print(f"  Water:    {water.sum():>8,} px")

        # ── 2. 道路 ──
        road, road_lvl_vals = self._classify_road(r, g, b, opaque, water)
        class_id[road] = 20
        road_lvl[road] = road_lvl_vals[road]
        road_highway  = road_lvl == 40
        road_national = road_lvl == 80
        road_province = road_lvl == 120
        road_branch   = road_lvl == 160
        road_path     = road_lvl == 200
        print(f"  Road:     {road.sum():>8,} px ({100*road.sum()/opaque.sum():.2f}%)")
        print(f"    - 高速:  {road_highway.sum():>8,} px")
        print(f"    - 国道:  {road_national.sum():>8,} px")
        print(f"    - 省道:  {road_province.sum():>8,} px")
        print(f"    - 支路:  {road_branch.sum():>8,} px")
        print(f"    - 小路:  {road_path.sum():>8,} px")

        # ── 3. 建筑 ──
        building = self._classify_building(r, g, b, opaque, water, road)
        class_id[building] = 40
        height[building] = 120
        print(f"  Building: {building.sum():>8,} px")

        # ── 4. 地面 ──
        ground = self._classify_ground(r, g, b, opaque)
        class_id[ground] = 60
        ground_count = (opaque & (class_id == 60)).sum()
        print(f"  Ground:   {ground_count:>8,} px")

        # 透明区域（缺失瓦片）→ 地面
        class_id[~opaque] = 60
        print(f"  Total:    {opaque.sum():>8,} opaque + {(~opaque).sum():>8,} transparent")

        # ── 保存 ──
        self._save_outputs(water, building, road, class_id, road_lvl, height)

    def _save_outputs(self, water, building, road, class_id, road_lvl, height):
        """保存为 GeoTIFF,对齐卫星图"""
        with rasterio.open(self.reference_sat) as src:
            crs, tf = src.crs, src.transform
            sW, sH = src.width, src.height

        def resize(arr):
            img = Image.fromarray(
                (arr * 255).astype(np.uint8) if arr.dtype == bool else arr.astype(np.uint8),
                mode="L"
            )
            return np.array(img.resize((sW, sH), Image.NEAREST), dtype=np.uint8)

        profile = {
            "driver": "GTiff", "height": sH, "width": sW,
            "crs": crs, "transform": tf, "compress": "lzw",
            "dtype": "uint8"
        }

        for path, data, count in [
            (self.water_out,    resize(water),    1),
            (self.building_out, resize(building), 1),
            (self.road_out,     resize(road),     1),
        ]:
            profile.update(count=count)
            with rasterio.open(path, "w", **profile) as dst:
                dst.write(data.reshape(sH, sW) if data.ndim == 1 else data, 1)
            print(f"  Saved {Path(path).name}")

        profile.update(count=3)
        with rasterio.open(self.label_out, "w", **profile) as dst:
            dst.write(resize(class_id), 1)
            dst.write(resize(road_lvl), 2)
            dst.write(resize(height),   3)
        print(f"  Saved {Path(self.label_out).name}")


# ============ 独立测试入口 / CLI ============

def test(vec_path: str = None, sat_path: str = None, out_dir: str = None):
    """检测分类器输入是否有效，无效则报错，有效则运行。"""
    clf = VecClassifier()
    vec = vec_path or "TestInput/SanHe/vec_raw.png"
    sat = sat_path or "TestInput/SanHe/satellite.tif"
    out = out_dir or "TestInput/SanHe"
    clf.set_input(vec, sat)
    clf.set_output(out)
    clf._check_paths()
    clf.run()


def main():
    p = argparse.ArgumentParser(description="天地图 vec_w 语义标签分类器")
    p.add_argument("--vec", default="TestInput/SanHe/vec_raw.png", help="vec_w 拼接图路径")
    p.add_argument("--sat", default="TestInput/SanHe/satellite.tif", help="卫星图参考路径")
    p.add_argument("--out-dir", default="TestInput/SanHe", help="输出目录")
    args = p.parse_args()
    test(args.vec, args.sat, args.out_dir)


if __name__ == "__main__":
    main()

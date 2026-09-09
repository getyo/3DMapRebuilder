#!/usr/bin/env python3
"""
共享几何常量与坐标换算 — 3DMapRebuilder

同一份输入（10x 语义标签 + DEM + 像素尺寸常量）会被多个模块各自换算成
UE 世界坐标：

  - gen_adaptive_terrain_centerline.py   地形 OBJ 导出（导入 UE 后 Y 翻转）
  - gen_water_flow.py                    中心线 CSV / 速度场纹理

坐标换算不一致会直接导致 UE 里中心线 / 速度场与水面 mesh 错位（此前
CSV 的 Y 偏移就踩过坑）。因此常量与换算集中在本文文件，各模块禁止
另写一份。

坐标系约定（与 OBJ 导出一致）：
  - 以整幅 10x 图像中心为原点。
  - OBJ 文件内为右手系 Z-up：x=east，y=-north，z=height。
  - UE 导入 OBJ 时翻转 Y（右手→左手），mesh 在 UE 中 y=+south。
  - 中心线 CSV 直接作为 UE 世界坐标读入，故其 y = -(OBJ 内 y)。

说明：
  - SCALE = 10 为 10x 上采样倍数；10x 像素坐标 = 原始像素坐标 × 10。
  - 速度场纹理与水面 OBJ 的 UV 都定义在"整幅 10x 图像空间"上，与上述
    世界坐标换算相互独立。
  - gen_semantic.py 暂未迁移到本文件，其 PX_M 等为自持副本，后续如需
    统一再收敛。
"""

PX_M = 0.458      # 原始(1x)像素地面尺寸 (m)
SCALE = 10        # 标签 10x 上采样倍数
UE_SCALE = 100.0  # m → UE 单位 的缩放

# 1 个 10x 像素对应的 UE 单位长度
UE_PER_PX10 = (PX_M * UE_SCALE) / SCALE
# 1 个 UE 单位对应的 10x 像素数
PX10_PER_UE = 1.0 / UE_PER_PX10


def ue_x_of_col(c: float, w10: int) -> float:
    """10x 列坐标 → UE 世界 X（east），以图中心为原点。"""
    return (c - w10 * 0.5) * UE_PER_PX10


def ue_y_obj_of_row(r: float, h10: int) -> float:
    """10x 行坐标 → OBJ 文件内 y（右手系 y=-north），OBJ 导出用。"""
    return (h10 * 0.5 - r) * UE_PER_PX10


def ue_y_csv_of_row(r: float, h10: int) -> float:
    """10x 行坐标 → 中心线 CSV 的 y（UE 左手系 y=+south），CSV 直读用。"""
    return (r - h10 * 0.5) * UE_PER_PX10

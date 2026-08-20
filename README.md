# 3DMapRebuilder

从对齐的天地图卫星影像、天地图矢量底图和高程数据生成 3DGS 语义地图，并导出供 UE 使用的自适应分辨率地形 OBJ。

测试区域：河北三河鲍邱河附近农村区，约 0.47×0.47 km（117.0991°E, 39.8349°N 为中心），面积约 0.22 km²。

## 一、数据源

| 数据 | 来源 | 规格 | 说明 |
|------|------|------|------|
| **天地图卫星图** | 天地图 WMTS `img_w` zoom 18 | ~0.46m/px, EPSG:4326 | 256×256 瓦片拼接为 GeoTIFF |
| **天地图矢量图** | 天地图 WMTS `vec_w` zoom 18 | ~0.46m/px, RGBA 瓦片 | 含语义颜色编码（水体/建筑/道路/地面） |
| **DEM 高程** | SRTM1 v3.0 (NASA) | 30m 级别（1 弧秒） | 三次立方插值升采样至卫星图分辨率 |

> **注意**：本仓库代码不包含输入数据的地理对齐和插值步骤。天地图瓦片下载、SRTM1 拼接、DEM 三次插值、卫星图与矢量图对齐等预处理需另行完成，当前直接使用已处理好的输入文件。且测试输入数据并不为全部数据源，只用了 16 个瓦片大小，大约 0.22 平方千米。

## 二、总体流程

完整技术管线为：

```
分类 → 后处理 → AI 生成 3DGS 地图 → 转化为 OBJ → UE 制作语义层
```

当前阶段出于简化和输入数据不足（只有卫星图，缺乏三维信息）的原因，先制作**简化的 3DGS 语义地图**（`gen_semantic.py`），并新增**自适应分辨率地形 OBJ 生成**（`gen_adaptive_terrain.py`），直接供 UE 使用。

未来输入数据具备真实三维信息后，可用卫星图直出完整 3DGS 渲染点云，替换当前简化版语义地图。

## 三、系统架构

```
输入层（预处理，外部完成）
  │
  ├── 天地图卫星图 (satellite.tif)
  ├── 天地图矢量图 (vec_raw.png)
  └── 高程图 (dem.tif)
  │
  ▼
┌─────────────────────────────────────────────────────────────┐
│ Stage 1: 语义分类                                            │
│ classify_vecw.py :: VecClassifier + main()                   │
│ 从 vec_raw.png 的 RGBA 颜色分类语义标签                       │
│ 输出: water.tif, building.tif, road.tif, labels.tif         │
└─────────────────────────────────────────────────────────────┘
  │
  ▼
┌─────────────────────────────────────────────────────────────┐
│ Stage 2: 标签后处理                                          │
│ label_postprocess.py :: LabelPostprocessor + main()          │
│ 对 labels.tif 做 10x 上采样、轮廓平滑、边界提取              │
│ 输出: labels_10x_no_boundary.tif, boundary_lines_10x.png    │
└─────────────────────────────────────────────────────────────┘
  │
  ├───▶┌─────────────────────────────────────────────────────┐
  │    │ Stage 3: 简化 3DGS 语义地图                          │
  │    │ gen_semantic.py :: SemanticMapBuilder + main()       │
  │    │ 从 labels.tif + dem 生成 3DGS 语义点云 PLY           │
  │    │ 输出: semanticMap.ply                                │
  │    └─────────────────────────────────────────────────────┘
  │
  ▼
┌─────────────────────────────────────────────────────────────┐
│ Stage 4: 自适应地形 OBJ（推荐入口）                          │
│ gen_adaptive_terrain.py :: AdaptiveTerrainBuilder + main()   │
│ 从 10x 标签 + 边界线 + DEM 生成 UE 可用 OBJ                  │
│ 输出: terrain_ground.obj, terrain_water.obj, terrain_building.obj │
│       terrain_water_centerline.csv, terrain_water_centerline_preview.png │
└─────────────────────────────────────────────────────────────┘
  │
  ▼
┌─────────────────────────────────────────────────────────────┐
│ Stage 5: UE 语义层（规划中）                                 │
│ 材质系统、语义查询、区域过滤、交互编辑等上层应用             │
└─────────────────────────────────────────────────────────────┘
```

## 四、模块输入输出

| 模块 | 核心类型 | 主要输入 | 主要输出 |
|------|----------|----------|----------|
| **VecClassifier** | 语义分类器 | `vec_raw.png`, `satellite.tif` | `water.tif`, `building.tif`, `road.tif`, `labels.tif` |
| **LabelPostprocessor** | 标签后处理器 | `labels.tif` | `labels_10x_no_boundary.tif`, `boundary_lines_10x.png`, `preview_classification_noboundary.png` |
| **SemanticMapBuilder** | 3DGS 点云生成器 | `labels.tif`, `dem.tif` | `semanticMap.ply` |
| **AdaptiveTerrainBuilder** | 自适应地形生成器 | `labels_10x_no_boundary.tif`, `boundary_lines_10x.png`, `dem.tif` | `terrain_ground.obj`, `terrain_water.obj`, `terrain_building.obj`, `terrain_water_centerline.csv`, `terrain_water_centerline_preview.png` |

## 五、使用说明

### 5.1 一键跑完整流程

```bash
python gen_adaptive_terrain.py
```

会依次执行：
1. `classify_vecw` 语义分类
2. `label_postprocess` 标签后处理
3. `gen_semantic` 简化 3DGS 语义地图
4. `AdaptiveTerrainBuilder.build()` 自适应地形 OBJ

### 5.2 强制重新生成所有中间文件

```bash
python gen_adaptive_terrain.py --force
```

默认情况下会利用已有中间文件，只有缺失或指定 `--force` 时才重新生成。

### 5.3 单独运行各模块

四个模块均可独立运行，均提供命令行入口，支持通过参数覆盖默认路径：

```bash
# 语义分类
python classify_vecw.py --vec TestInput/SanHe/vec_raw.png --sate TestInput/SanHe/satellite.tif --out TestInput/SanHe

# 标签后处理
python label_postprocess.py --input TestInput/SanHe/labels.tif --out-dir TestInput/SanHe/Output_10x

# 3DGS 语义地图
python gen_semantic.py --label TestInput/SanHe/labels.tif --dem TestInput/SanHe/dem.tif --out TestInput/SanHe/semanticMap.ply

# 自适应地形（完整管线）
python gen_adaptive_terrain.py --label-dir TestInput/SanHe --dem TestInput/SanHe/dem.tif --out-dir output/terrain_adaptive
```

也可通过 `test()` 函数以编程方式调用：

```python
from classify_vecw import test
test(vec_path='...', sate_path='...', out_dir='...')

from label_postprocess import test
test(input_path='...', output_dir='...')

from gen_semantic import test
test(label_path='...', dem_path='...', out_path='...')
```

### 5.4 命令行参数

```bash
python gen_adaptive_terrain.py --help
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--label-dir` | `TestInput/SanHe` | 输入标签目录 |
| `--dem` | `TestInput/SanHe/dem.tif` | DEM 路径 |
| `--out-dir` | `output/terrain_adaptive` | 地形 OBJ 输出目录 |
| `--boundary-step` | `1` | 边界采样步长（10x 像素） |
| `--force` | `False` | 强制重新生成所有中间文件 |

## 六、主要类型说明

### 6.1 VecClassifier

`classify_vecw.py` 中的类，将天地图矢量底图按颜色分类。

**输出格式（labels.tif 三通道）**：

| 通道 | 内容 | 值范围 |
|------|------|--------|
| Band 1 (class_id) | 语义类别 | 0=水体, 20=道路, 40=建筑, 60=地面 |
| Band 2 (road_level) | 道路等级 | 40=高速, 80=国道, 120=省道, 160=支路, 200=小路 |
| Band 3 (height) | 相对高度（占位） | 120=建筑, 0=其他 |

### 6.2 LabelPostprocessor

`label_postprocess.py` 中的类，对 `labels.tif` 做 10x 上采样与轮廓平滑。

### 6.3 SemanticMapBuilder

`gen_semantic.py` 中的类，从语义标签 + DEM 生成 3DGS PLY 点云。

### 6.4 AdaptiveTerrainBuilder

`gen_adaptive_terrain.py` 中的类，从 10x 标签 + 边界线 + DEM 生成 UE 可用的自适应分辨率地形 OBJ。

**新增能力**：
- 从水体掩膜提取河流中心线，输出 CSV 供 UE DataTable 导入，支持水体扩散/流动系统的后续开发。
- 调用 `velocity_field.py` 生成静态速度场纹理，作为后续 UE 材质/Niagara 的流动方向输入。

**核心策略**：
- 粗网格 stride = 10 原始像素（100 10x 像素）。
- 从 `boundary_lines_10x.png` 细化出 1 像素宽 10x 边界线。
- 粗网格顶点 + 密集边界点用 `scipy.spatial.Delaunay` 三角化，三角形大小自然自适应：内部大、边界密。
- 每个三角形取重心位置的 10x class_id，分配到 `M_Ground` / `M_Road` / `M_Building` / `M_Water` 四个 UE 材质槽。
- 水体通过 `distance_transform_edt` 生成凹包盆地，保留水边过渡。
- 坐标系为 UE 左手系 Z-up：`X=east, Y=-north, Z=height`。

**设计特点**：

- 纯数值运算（重心计算、顶点生成、法线计算等）采用 NumPy 向量化实现，保留逻辑密集环节（裙边检测、面分配）的显式循环以确保可维护性。
- OBJ 导出采用批量字符串拼接，减少小写入开销。

## 七、输入文件

`TestInput/SanHe/` 下需要预先准备：

| 文件 | 尺寸 | 说明 |
|------|------|------|
| `satellite.tif` | 1024×1024 | 天地图 RGB 卫星图，EPSG:4326，~0.46m/px |
| `vec_raw.png` | 1024×1024 | 天地图 vec_w RGBA 拼接图，4×4 = 16 张瓦片（zoom 18，256px/片） |
| `dem.tif` | 1024×1024 | float32 高程，与卫星图对齐，30m SRTM1 三次插值 |

## 八、输出文件

- `TestInput/SanHe/water.tif` — 水体二值掩膜
- `TestInput/SanHe/building.tif` — 建筑二值掩膜
- `TestInput/SanHe/road.tif` — 道路二值掩膜
- `TestInput/SanHe/labels.tif` — 三通道语义标签
- `TestInput/SanHe/semanticMap.ply` — 3DGS 语义点云
- `TestInput/SanHe/Output_10x/labels_10x_no_boundary.tif` — 10x 上采样语义标签
- `TestInput/SanHe/Output_10x/boundary_lines_10x.png` — 独立轮廓线图
- `TestInput/SanHe/Output_10x/preview_classification_noboundary.png` — 分类预览图
- `output/terrain_adaptive/terrain_ground.obj` — 地面/道路/水体凹包 + 建筑裙边
- `output/terrain_adaptive/terrain_ground.mtl` — 地面材质定义
- `output/terrain_adaptive/terrain_water.obj` — 蓝色水面盖子
- `output/terrain_adaptive/terrain_water.mtl` — 水面材质定义
- `output/terrain_adaptive/terrain_building.obj` — 建筑核心区域顶面
- `output/terrain_adaptive/terrain_building.mtl` — 建筑材质定义
- `output/terrain_adaptive/terrain_water_centerline.csv` — 河流中心线点序列（UE DataTable 格式）
- `output/terrain_adaptive/terrain_water_centerline_preview.png` — 水体掩膜 + 红色中心线预览
- `output/terrain_adaptive/terrain_velocity_field.png` — 水体静态速度场纹理（RG=方向，B=有效掩膜）
- `output/terrain_adaptive/terrain_velocity_field_preview.png` — 速度场流向可视化预览

## 九、水体扩散实现管线

本章节说明当前已接入的水体扩散相关流程，以及后续与 UE 对接的步骤。

```
Python 端
  │
  ├── 输入：天地图矢量图 → classify_vecw.py → labels.tif
  │
  ├── label_postprocess.py → labels_10x_no_boundary.tif（10x 水体掩膜）
  │
  └── gen_adaptive_terrain.py
        │
        ├── 生成 terrain_water.obj（水面网格，供 UE 渲染）
        ├── 生成 terrain_water_centerline.csv（河流中心线，UE DataTable 导入）
        ├── 生成 terrain_water_centerline_preview.png（人工校验）
        └── 调用 velocity_field.py 生成 terrain_velocity_field.png（静态速度场）

UE 端
  │
  ├── 导入 terrain_water.obj 为 Static Mesh
  │
  ├── 创建蓝图结构体 FCenterlinePoint（x, y, z）
  │
  ├── 导入 CSV 为 DataTable（Row Type = FCenterlinePoint）
  │
  ├── 在 BP_RiverWater 蓝图中：
  │     For Each Loop 读取 DataTable → Break FCenterlinePoint
  │     → Make Vector → Add Spline Point
  │     生成沿河流中心的 Spline
  │
  ├── 扩散 RT 模拟：
  │     扩散 Grid 的每个格子通过世界坐标/水体 UV 采样速度场 Texture，
  │     用速度方向做 Semi-Lagrangian Advection，实现污染物随水流扩散。
  │
  └── 如需动态/更复杂河岸插值，可在 UE 中按 Spline 实时生成速度场 RT 替代静态纹理。
```

## 十、待办

- [ ] Stage 5：UE 语义层（材质系统、语义查询、区域过滤、交互编辑）
- [ ] 水体污染扩散 Grid2D 与 UE 材质联动
- [ ] 完整版 3DGS：从卫星图直出完整 3DGS 渲染点云（需补充三维信息输入）
- [ ] labels.tif 第三通道补入真实建筑高度（GBA 或阴影法）
- [ ] 多测试区域支持

# 3DMapRebuilder

从对齐的天地图卫星影像、天地图矢量底图和高程数据生成 3DGS 语义地图，并导出供 UE 使用的自适应分辨率地形 OBJ。

测试区域：河北三河鲍邱河附近农村区，约 0.47×0.47 km（117.0991°E, 39.8349°N 为中心），面积约 0.22 km²。

## 数据源

| 数据 | 来源 | 规格 | 说明 |
|------|------|------|------|
| **天地图卫星图** | 天地图 WMTS `img_w` zoom 18 | ~0.46m/px, EPSG:4326 | 256×256 瓦片拼接为 GeoTIFF |
| **天地图矢量图** | 天地图 WMTS `vec_w` zoom 18 | ~0.46m/px, RGBA 瓦片 | 含语义颜色编码（水体/建筑/道路/地面） |
| **DEM 高程** | SRTM1 v3.0 (NASA) | 30m 级别（1 弧秒） | 三次立方插值升采样至卫星图分辨率 |

> **注意**：本仓库代码不包含输入数据的地理对齐和插值步骤。天地图瓦片下载、SRTM1 拼接、DEM 三次插值、卫星图与矢量图对齐等预处理需另行完成，当前直接使用已处理好的输入文件。且测试输入数据并不为全部数据源，只用了 16 个瓦片大小，大约 0.22 平方千米。

## 总体流程

完整技术管线为：

```
分类 → 后处理 → AI 生成 3DGS 地图 → 转化为 OBJ → UE 制作语义层
```

当前阶段出于简化和输入数据不足（只有卫星图，缺乏三维信息）的原因，先制作**简化的 3DGS 语义地图**（`gen_semantic.py`），并新增**自适应分辨率地形 OBJ 生成**（`gen_adaptive_terrain.py`），直接供 UE 使用。

未来输入数据具备真实三维信息后，可用卫星图直出完整 3DGS 渲染点云，替换当前简化版语义地图。

## 系统架构

```
输入层（预处理，外部完成）
  │
  ├── 天地图卫星图 (satellite.tif)
  ├── 天地图矢量图 (vec_raw.png)
  └── 高程图 (dem.tif)
  │
  ▼
┌─────────────────────────────────────────────────────────────┐
│ Stage 1: 语义分类（已完成 ✓）                                 │
│ classify_vecw.py :: VecClassifier                             │
│ 从 vec_raw.png 的 RGBA 颜色分类语义标签                        │
│ 输出: water.tif, building.tif, road.tif, labels.tif          │
└─────────────────────────────────────────────────────────────┘
  │
  ▼
┌─────────────────────────────────────────────────────────────┐
│ Stage 2: 标签后处理（已完成 ✓）                               │
│ label_postprocess.py :: LabelPostprocessor                    │
│ 对 labels.tif 做 10x 上采样、轮廓平滑、边界提取                │
│ 输出: labels_10x_no_boundary.tif, boundary_lines_10x.png     │
└─────────────────────────────────────────────────────────────┘
  │
  ├───▶┌─────────────────────────────────────────────────────┐
  │    │ Stage 3: 简化 3DGS 语义地图（已完成 ✓）              │
  │    │ gen_semantic.py :: SemanticMapBuilder                 │
  │    │ 从 labels.tif + dem 生成 3DGS 语义点云 PLY            │
  │    │ 输出: semanticMap.ply                                 │
  │    └─────────────────────────────────────────────────────┘
  │
  ▼
┌─────────────────────────────────────────────────────────────┐
│ Stage 4: 自适应地形 OBJ（已完成 ✓）                           │
│ gen_adaptive_terrain.py :: AdaptiveTerrainBuilder             │
│ 从 10x 标签 + 边界线 + DEM 生成 UE 可用 OBJ                   │
│ 输出: terrain_adaptive.obj, terrain_adaptive.mtl             │
└─────────────────────────────────────────────────────────────┘
  │
  ▼
┌─────────────────────────────────────────────────────────────┐
│ Stage 5: UE 语义层（规划中）                                  │
│ 材质系统、语义查询、区域过滤、交互编辑等上层应用                │
└─────────────────────────────────────────────────────────────┘
```

## 模块输入输出

| 模块 | 核心类型 | 主要输入 | 主要输出 |
|------|----------|----------|----------|
| **VecClassifier** | 语义分类器 | `vec_raw.png`, `satellite.tif` | `water.tif`, `building.tif`, `road.tif`, `labels.tif` |
| **LabelPostprocessor** | 标签后处理器 | `labels.tif` | `labels_10x_no_boundary.tif`, `boundary_lines_10x.png`, `preview_classification_noboundary.png` |
| **SemanticMapBuilder** | 3DGS 点云生成器 | `labels.tif`, `dem.tif` | `semanticMap.ply` |
| **AdaptiveTerrainBuilder** | 自适应地形生成器 | `labels_10x_no_boundary.tif`, `boundary_lines_10x.png`, `dem.tif` | `terrain_adaptive.obj`, `terrain_adaptive.mtl` |

## 当前数据流

```
vec_raw.png (RGBA 矢量底图)         satellite.tif (RGB 卫星图基准)    dem.tif (高程, 对齐)
       │                                        │                        │
       │  classify_vecw.py                       │                        │
       ▼                                        │                        │
  ┌──────────┐                                  │                        │
  │ 颜色分类  │  ← 8 种 RGBA 颜色常量 (±1 容差)    │                        │
  └────┬─────┘                                  │                        │
       │                                         │                        │
       ├──→ water.tif (二值掩膜, class_id=0)      │                        │
       ├──→ road.tif (二值掩膜, class_id=20)      │                        │
       ├──→ building.tif (二值掩膜, class_id=40)  │                        │
       └──→ labels.tif (3 通道语义标签) ────────────┼──────────── dem.tif ──┤
                │   Band 1: class_id              │                        │
                │   Band 2: road_level            │                        │
                │   Band 3: height(占位)          │                        │
                │                                 │                        │
                ▼                                 │                        │
          label_postprocess.py ←──────────────────┘                        │
                │                                                          │
                │  · 10x 最近邻上采样                                       │
                │  · 水体/建筑高斯轮廓平滑                                  │
                │  · 道路冻结拐角 + 段内 Chaikin 插值                       │
                │  · 道路等级继承 / 建筑高度占位                            │
                ▼                                                          │
   labels_10x_no_boundary.tif + boundary_lines_10x.png                     │
                │                                                          │
                ├──→ gen_semantic.py ←─────────────────────────────────────┘
                │           │
                │           │  · 像素→米坐标转换（影像中心原点）
                │           │  · 建筑高度分配（3~10m, 面积自适应）
                │           │  · 语义边界/建筑拐角检测
                │           │  · 建筑墙面垂直分段高斯
                │           │  · 语义颜色 → 球谐 DC 系数
                │           ▼
                │   semanticMap.ply
                │   （65 floats/vertex, 260 bytes/vertex）
                │
                └──→ gen_adaptive_terrain.py
                            │
                            │  · 从 boundary_lines_10x.png 提取 10x 边界线
                            │  · 粗网格 + 密集边界点 → scipy Delaunay
                            │  · 三角形自适应：内部大、边界小
                            │  · 按重心 class 分 4 个 UE 材质槽
                            │  · 水体保留凹包/盆地
                            │  · 坐标系：UE 左手系 Z-up（X=east, Y=-north, Z=height）
                            ▼
                terrain_adaptive.obj + terrain_adaptive.mtl
```

## 主要类型与用法

### VecClassifier（语义分类器）

`classify_vecw.py` 中的单例类，将天地图矢量底图按颜色分类。

```python
from classify_vecw import VecClassifier

clf = VecClassifier()
clf.set_input("TestInput/SanHe/vec_raw.png", "TestInput/SanHe/satellite.tif")
clf.set_output("TestInput/SanHe")
clf.run()
# 输出: water.tif, building.tif, road.tif, labels.tif
```

**输出格式（labels.tif 三通道）**：

| 通道 | 内容 | 值范围 |
|------|------|--------|
| Band 1 (class_id) | 语义类别 | 0=水体, 20=道路, 40=建筑, 60=地面 |
| Band 2 (road_level) | 道路等级 | 40=高速, 80=国道, 120=省道, 160=支路, 200=小路 |
| Band 3 (height) | 相对高度（占位） | 120=建筑, 0=其他 |

**分类优先级**：水体 > 道路 > 建筑（形态闭合） > 地面 > 透明区域（默认地面）

**颜色常量**（天地图 vec_w zoom 18 的 RGBA 像素值）：

| 类别 | R | G | B |
|------|---|---|---|
| 水体 | 171 | 198 | 239 |
| 高速 | 186 | 160 | 241 |
| 国道 | 254 | 205 | 120 |
| 省道 | 254 | 235 | 130 |
| 支路 | 255 | 255 | 255 |
| 小路 | 253 | 253 | 253 |
| 建筑 | 249 | 250 | 243 |
| 地面 | 245 | 244 | 238 |

### SemanticMapBuilder（语义地图构建器）

`gen_semantic.py` 中的单例类，从语义标签 + DEM 生成 3DGS PLY 点云。

```python
from gen_semantic import SemanticMapBuilder

# 方式 1：默认路径（TestInput/SanHe/）
builder = SemanticMapBuilder()
builder.build()

# 方式 2：自定义路径
builder = SemanticMapBuilder(
    label_path="path/to/labels.tif",
    dem_path="path/to/dem.tif",
    out_path="path/to/output.ply"
)
builder.build()

# 方式 3：链式设置
builder = SemanticMapBuilder()
builder.set_input(
    label_path="path/to/labels.tif",
    dem_path="path/to/dem.tif",
    out_path="path/to/output.ply"
).build()
```

**几何参数**（可在类静态属性中修改）：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `PX_M` | 0.458 | 像素地面尺寸 (m) |
| `BUILD_MIN_H` | 3.0 | 建筑最小高度 (m) |
| `BUILD_MAX_H` | 10.0 | 建筑最大高度 (m) |
| `SIGMA_H` | 0.7 | 面内部水平 σ |
| `SIGMA_CONTOUR` | 0.001 | 轮廓 σ（锐边） |
| `WALL_STEP` | 0.5 | 墙面垂直分段 (m) |
| `WALL_THICK_NORMAL` | 0.5 | 墙面法向 σ |

### PLYFile（PLY 格式定义）

`gen_semantic.py` 中的纯数据类，定义 3DGS PLY 的字段布局。

每个顶点 65 个 float32 字段（260 字节）：

| 偏移 | 字段 | 数量 | 说明 |
|------|------|------|------|
| 0–23 | x, y, z, nx, ny, nz | 6 | 位置与法线（y-up） |
| 24–35 | f_dc_0, f_dc_1, f_dc_2 | 3 | 球谐 DC 颜色系数 |
| 36–215 | f_rest_0..44 | 45 | 球谐高阶系数 |
| 216–219 | opacity | 1 | logit 不透明度 |
| 220–231 | scale_0, scale_1, scale_2 | 3 | log(σ) 尺度 |
| 232–247 | rot_0..3 | 4 | 旋转四元数 |
| 248–251 | semantic_class | 1 | 语义类别（float） |
| 252–255 | ground_height | 1 | 地面高程 |
| 256–259 | relative_height | 1 | 相对高度 |

**语义映射**：water=0, ground=1, building=2, road=3。

**墙体颜色**：橙色（屋顶色×0.5，拐角处缩至轮廓 σ 保持锐利）。

### LabelPostprocessor（标签后处理器）

`label_postprocess.py` 中的非单例类，对分类器输出的 `labels.tif` 做 10x 上采样与轮廓平滑。

```python
from label_postprocess import LabelPostprocessor

# 方式 1：默认路径
pp = LabelPostprocessor()
pp.process()

# 方式 2：构造时指定路径
pp = LabelPostprocessor(
    input_path="path/to/labels.tif",
    output_dir="path/to/Output_10x"
)
pp.process()

# 方式 3：链式设置
pp = LabelPostprocessor()
pp.set_input("path/to/labels.tif").set_output("path/to/Output_10x").process()
```

**输入**：分类器输出的 `labels.tif`（三通道语义标签）。  
**输出**：
- `labels_10x_no_boundary.tif`：10x 上采样后的三通道语义标签
- `boundary_lines_10x.png`：RGBA 独立轮廓线图
- `preview_classification_noboundary.png`：分类预览图

### AdaptiveTerrainBuilder（自适应地形生成器）

`gen_adaptive_terrain.py` 中的非单例类，从 10x 标签 + 边界线 + DEM 生成 UE 可用的自适应分辨率地形 OBJ。

```python
from gen_adaptive_terrain import AdaptiveTerrainBuilder

builder = AdaptiveTerrainBuilder(
    label_10x_path="path/to/labels_10x_no_boundary.tif",
    boundary_path="path/to/boundary_lines_10x.png",
    dem_path="path/to/dem.tif",
    out_dir="path/to/terrain_adaptive",
)
builder.build()
```

**输入**：
- `labels_10x_no_boundary.tif`：10x 语义标签（Band 1 为 class_id）
- `boundary_lines_10x.png`：RGBA 轮廓线图（alpha > 0 为边界）
- `dem.tif`：高程图

**输出**：
- `terrain_adaptive.obj`：三角网格，按 4 个材质槽分组
- `terrain_adaptive.mtl`：材质定义（含区分色）

**核心策略**：
- 粗网格 stride = 10 原始像素（100 10x 像素）。
- 从 `boundary_lines_10x.png` 细化出 1 像素宽 10x 边界线。
- 粗网格顶点 + 密集边界点用 `scipy.spatial.Delaunay` 三角化，三角形大小自然自适应：内部大、边界密。
- 每个三角形取重心位置的 10x class_id，分配到 `M_Ground` / `M_Road` / `M_Building` / `M_Water` 四个 UE 材质槽。
- 水体通过 `distance_transform_edt` 生成凹包盆地，保留水边过渡。
- 坐标系为 UE 左手系 Z-up：`X=east, Y=-north, Z=height`。

## 输入文件

`TestInput/SanHe/` 下需要预先准备：

| 文件 | 尺寸 | 说明 |
|------|------|------|
| `satellite.tif` | 1024×1024 | 天地图 RGB 卫星图，EPSG:4326，~0.46m/px |
| `vec_raw.png` | 1024×1024 | 天地图 vec_w RGBA 拼接图，4×4 = 16 张瓦片（zoom 18，256px/片） |
| `dem.tif` | 1024×1024 | float32 高程，与卫星图对齐，30m SRTM1 三次插值 |

> 对齐参数：CRS=EPSG:4326, DEM 插值用 cubic, 标签用 nearest。测试范围 117.0964~117.1019°E, 39.8328~39.8370°N，面积约 0.22 km²。

## 运行方式

```bash
pip install numpy pillow rasterio scipy plyfile opencv-python
python label_postprocess.py   # 一键跑完全流程：分类 → 后处理 → 3DGS语义地图 → 自适应地形OBJ
```

`label_postprocess.py` 的 `main()` 会依次执行：
1. `VecClassifier.run()` 生成语义标签
2. `LabelPostprocessor.process()` 生成 10x 上采样标签与轮廓图
3. `SemanticMapBuilder.build()` 生成 3DGS PLY
4. `AdaptiveTerrainBuilder.build()` 生成自适应地形 OBJ

如需单独执行某一阶段，可直接导入对应类：

```python
from classify_vecw import VecClassifier
from gen_semantic import SemanticMapBuilder
from label_postprocess import LabelPostprocessor
from gen_adaptive_terrain import AdaptiveTerrainBuilder
```

## 输出

- `TestInput/SanHe/water.tif` — 水体二值掩膜
- `TestInput/SanHe/building.tif` — 建筑二值掩膜（形态闭合）
- `TestInput/SanHe/road.tif` — 道路二值掩膜（五级合并）
- `TestInput/SanHe/labels.tif` — 三通道语义标签
- `TestInput/SanHe/semanticMap.ply` — 3DGS 语义点云（SIBR / gsplat viewer 可直接查看）
- `TestInput/SanHe/Output_10x/labels_10x_no_boundary.tif` — 10x 上采样语义标签
- `TestInput/SanHe/Output_10x/boundary_lines_10x.png` — 独立轮廓线图
- `TestInput/SanHe/Output_10x/preview_classification_noboundary.png` — 分类预览图
- `TestInput/SanHe/Output_10x/terrain_adaptive/terrain_adaptive.obj` — UE 自适应地形网格
- `TestInput/SanHe/Output_10x/terrain_adaptive/terrain_adaptive.mtl` — UE 地形材质定义

## 待办

- [ ] Stage 5：UE 语义层（材质系统、语义查询、区域过滤、交互编辑）
- [ ] 第三层完整版：从卫星图直出完整 3DGS 渲染点云（需补充三维信息输入）
- [ ] labels.tif 第三通道补入真实建筑高度（GBA 或阴影法）
- [ ] 多测试区域支持

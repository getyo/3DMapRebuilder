# 3DMapRebuilder

从对齐的天地图卫星影像、天地图矢量底图和高程数据生成 3DGS 语义地图，并导出供 UE 使用的自适应分辨率地形 OBJ 与水体流场数据。

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

当前阶段出于简化和输入数据不足（只有卫星图，缺乏三维信息）的原因，先制作**简化的 3DGS 语义地图**（`gen_semantic.py`），并新增**自适应分辨率地形 OBJ 生成**（`gen_adaptive_terrain_centerline.py` 的地形构建部分）与**水体流场生成**（`gen_water_flow.py`，含水体中心线与速度场），直接供 UE 使用。

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
  ▼  Stage 3（三路并行，任一失败不影响其它，见 gen_adaptive_terrain_centerline.py）
┌──────────────────────┬──────────────────────┬──────────────────────┐
│ 3a: 简化 3DGS 语义地图 │ 3b: 自适应地形 OBJ    │ 3c: 水体流场           │
│ gen_semantic.py      │ gen_adaptive_terrain │ gen_water_flow.py     │
│ SemanticMapBuilder   │ _centerline.py       │ WaterFlowBuilder      │
│ labels.tif + dem     │ AdaptiveTerrainBuilder│ 10x 水体掩膜 + dem     │
│ → semanticMap.ply    │ 10x标签+边界+dem      │ → 中心线 CSV + 速度场   │
│                      │ → terrain_*.obj      │   纹理 + 预览图        │
└──────────────────────┴──────────────────────┴──────────────────────┘
  │
  ├── 坐标对齐校验：中心线 CSV 必须落在 terrain_water.obj 的 AABB 内
  │   （常量与换算统一在 map_common.py，防止两模块坐标漂移）
  ▼
┌─────────────────────────────────────────────────────────────┐
│ Stage 4: UE 语义层（规划中）                                 │
│ 材质系统、语义查询、区域过滤、交互编辑等上层应用             │
└─────────────────────────────────────────────────────────────┘
```

共享常量与坐标换算（`map_common.py`）：地形 OBJ 导出、中心线 CSV、速度场纹理都涉及「10x 像素 → UE 世界坐标」的换算，两处符号约定不同（OBJ 文件内为右手系 y=-north；UE 导入后 Y 翻转，CSV 直接以 UE 坐标 y=+south 读入）。此前 CSV Y 偏移正是因此产生，现统一由 `map_common.py` 提供常量与换算函数，并在地形/流场产物都就绪后做一次 AABB 对齐校验。

## 四、模块输入输出

| 模块 | 核心类型 | 主要输入 | 主要输出 |
|------|----------|----------|----------|
| **VecClassifier** | 语义分类器 | `vec_raw.png`, `satellite.tif` | `water.tif`, `building.tif`, `road.tif`, `labels.tif` |
| **LabelPostprocessor** | 标签后处理器 | `labels.tif` | `labels_10x_no_boundary.tif`, `boundary_lines_10x.png`, `preview_classification_noboundary.png` |
| **SemanticMapBuilder** | 3DGS 点云生成器 | `labels.tif`, `dem.tif` | `semanticMap.ply` |
| **AdaptiveTerrainBuilder** | 自适应地形生成器 | `labels_10x_no_boundary.tif`, `boundary_lines_10x.png`, `dem.tif` | `terrain_ground.obj`, `terrain_water.obj`, `terrain_building.obj`（含各自 .mtl） |
| **WaterFlowBuilder** | 水体流场生成器 | `labels_10x_no_boundary.tif`（band1==0 水体掩膜）, `dem.tif` | `terrain_water_centerline.csv`, `terrain_water_centerline_preview.png`, `terrain_velocity_field.png`, `terrain_velocity_field_preview.png` |
| **map_common** | 共享常量/坐标换算 | — | 各模块共享的像素尺寸、缩放、UE 坐标换算函数 |

## 五、使用说明

### 5.1 一键跑完整流程

```bash
python gen_adaptive_terrain_centerline.py
```

默认行为（缓存逻辑）：

1. `classify_vecw` 语义分类 —— `labels.tif` 已存在则跳过
2. `label_postprocess` 标签后处理 —— `labels_10x_no_boundary.tif` 与 `boundary_lines_10x.png` 已存在则跳过
3. 并行执行三路任务（各任务产物齐全则跳过，单路失败不中断其它路，失败详情见 `--out-dir/_logs/` 下对应日志）：
   - `gen_semantic` 简化 3DGS 语义地图
   - `AdaptiveTerrainBuilder` 地形 OBJ
   - `gen_water_flow` 水体中心线 + 速度场
4. 坐标对齐校验：中心线 CSV 是否落在水面 OBJ 的 AABB 内

### 5.2 强制重新生成

```bash
# 全部重新生成（含已有中间文件）
python gen_adaptive_terrain_centerline.py --force

# 只重新生成指定部分，其余按缓存逻辑
python gen_adaptive_terrain_centerline.py --regenerate flow
python gen_adaptive_terrain_centerline.py --regenerate terrain,flow
python gen_adaptive_terrain_centerline.py --regenerate classify --regenerate postprocess
```

### 5.3 单独运行各模块

各模块均可独立运行，均提供命令行入口，支持通过参数覆盖默认路径：

```bash
# 语义分类
python classify_vecw.py --vec TestInput/SanHe/vec_raw.png --sat TestInput/SanHe/satellite.tif --out-dir TestInput/SanHe

# 标签后处理
python label_postprocess.py --input TestInput/SanHe/labels.tif --out-dir TestInput/SanHe/Output_10x

# 3DGS 语义地图
python gen_semantic.py --label TestInput/SanHe/labels.tif --dem TestInput/SanHe/dem.tif --out TestInput/SanHe/semanticMap.ply

# 水体流场（中心线 + 速度场）
python gen_water_flow.py --label-10x TestInput/SanHe/Output_10x/labels_10x_no_boundary.tif --dem TestInput/SanHe/dem.tif --out-dir output/terrain_adaptive

# 地形 OBJ（单独跑地形，不触发完整管线）
python gen_adaptive_terrain_centerline.py --stage-terrain --label-dir TestInput/SanHe --dem TestInput/SanHe/dem.tif --out-dir output/terrain_adaptive
```

也可通过 `test()` 函数以编程方式调用：

```python
from classify_vecw import test
test(vec_path='...', sate_path='...', out_dir='...')

from label_postprocess import test
test(input_path='...', output_dir='...')

from gen_semantic import test
test(label_path='...', dem_path='...', out_path='...')

from gen_water_flow import test
test(label_10x_path='...', dem_path='...', out_dir='...')
```

### 5.4 命令行参数

```bash
python gen_adaptive_terrain_centerline.py --help
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--label-dir` | `TestInput/SanHe` | 输入标签目录 |
| `--dem` | `TestInput/SanHe/dem.tif` | DEM 路径 |
| `--out-dir` | `output/terrain_adaptive` | 地形 OBJ / 流场输出目录（并行任务日志在 `_logs/` 子目录） |
| `--boundary-step` | `1` | 边界采样步长（10x 像素） |
| `--velocity-res` | `2048` | 速度场纹理最长边像素数（传给 gen_water_flow） |
| `--velocity-seed` | `42` | Perlin 噪声随机种子（传给 gen_water_flow） |
| `--velocity-noise` | `0.15` | 流向噪声强度（弧度，传给 gen_water_flow） |
| `--force` | `False` | 强制重新生成所有产物（含已有中间文件） |
| `--regenerate` | `[]` | 只强制重新生成指定部分：`classify` / `postprocess` / `semantic` / `terrain` / `flow`，可重复指定或以逗号分隔 |

```bash
python gen_water_flow.py --help
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--label-10x` | `TestInput/SanHe/Output_10x/labels_10x_no_boundary.tif` | 10x 语义标签（水体 = band1 == 0） |
| `--dem` | `TestInput/SanHe/dem.tif` | DEM 路径 |
| `--out-dir` | `output/terrain_adaptive` | 输出目录 |
| `--velocity-res` | `2048` | 速度场纹理最长边像素数 |
| `--velocity-seed` | `42` | Perlin 噪声随机种子 |
| `--velocity-noise` | `0.15` | 流向噪声强度（弧度） |

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

`gen_adaptive_terrain_centerline.py` 中的类，从 10x 标签 + 边界线 + DEM 生成 UE 可用的自适应分辨率地形 OBJ（只负责地形网格，不生成水体流场数据）。

**坐标系**：
- OBJ 导出使用右手系 Z-up：`X=east, Y=-north, Z=height`。
- UE 导入 OBJ 时会做右手系→左手系转换，mesh 在 UE 里的实际坐标为 `X=east, Y=south, Z=height`。
- 常量与换算统一引用 `map_common.py`，与 `gen_water_flow.py` 保持一致。

**核心策略**：

1. 粗网格 stride = 10 原始像素（100 10x 像素）。
2. 从 `boundary_lines_10x.png` 细化出 1 像素宽 10x 边界线。
3. 粗网格顶点 + 密集边界点用 `scipy.spatial.Delaunay` 三角化，三角形大小自然自适应：内部大、边界密。
4. 每个三角形取重心位置的 10x class_id，分配到地面 OBJ 的 `M_Ground` / `M_Road` / `M_WaterBed` / `M_Building` 与水面 OBJ 的 `M_Water` 材质槽。
5. 水体通过 `distance_transform_edt` 生成凹包盆地，保留水边过渡。

### 6.5 WaterFlowBuilder

`gen_water_flow.py` 中的类，从 10x 水体掩膜（band1==0）+ DEM 生成水体中心线与静态速度场，是独立模块，与地形网格无数据依赖，可与地形 OBJ 并行执行。

**中心线提取**：
- 水体掩膜下采样到最长边 2048 后骨架化，取最大连通分量的直径路径（两次 BFS），高斯平滑后按弧长重采样（默认 500 UE 单位间距）。
- 输出 CSV 供 UE DataTable 导入（UE DataTable 格式，Row Type 含 FVector Position）。CSV 的 Y 直接以 UE 世界坐标 `y=+south` 输出，与 UE 导入 `terrain_water.obj` 后的 mesh Y 轴对齐。

**速度场生成**：
- 以中心线切线规定主体流向，岸边切线影响近岸区域，并叠加低频 Perlin 噪声模拟不规则流动；速度采用"中心快、岸边慢"的分布。
- 输出纹理为整幅 10x 图像空间、V-up 布局，可直接按水面 mesh 的 UV 采样。

### 6.6 map_common

`map_common.py` 提供共享常量（`PX_M` = 0.458 m/px、`SCALE` = 10、`UE_SCALE` = 100）与 10x 像素 → UE 世界坐标的换算函数（`ue_x_of_col` / `ue_y_obj_of_row` / `ue_y_csv_of_row`）。地形 OBJ 与水体流场必须引用同一份常量，禁止各自另写，防止 UE 内错位。

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
- `output/terrain_adaptive/terrain_velocity_field.png` — 水体静态速度场纹理（PNG，RGBA）：R=世界空间流向 X，G=世界空间流向 Y，B=速度大小（河道中心≈1，岸边→0，并叠加轻微噪声扰动），A=水体掩膜
- `output/terrain_adaptive/terrain_velocity_field_preview.png` — 速度场流向可视化预览（V-up 布局，与 UE 纹理一致）
- `output/terrain_adaptive/_logs/` — 并行阶段各任务的运行日志（`semantic.log` / `terrain.log` / `flow.log`）

## 九、速度场纹理设计与水体扩散管线

### 9.1 速度场纹理设计

速度场离线生成，导出 PNG 后导入 UE 作为 Texture2D，按水体 mesh 的 UV 直接采样。

**通道分配**：

| 通道 | 内容 | 说明 |
|------|------|------|
| **R** | 世界空间流向 X（-1~1 映射到 0~1） | 归一化方向，中心线切线 + 两岸加权 + Perlin 噪声 |
| **G** | 世界空间流向 Y（-1~1 映射到 0~1） | 同上 |
| **B** | 速度廓线系数（0~1） | 中心线附近 ≈ 1，岸边趋近 0 |
| **A** | 水体掩膜 | 1 = 有效水体区域，0 = 非水体/无效 |

**UV 对齐**：水体 mesh 的 UV AABB 为 `[0,1]×[0,1]`。速度场纹理按 V-up 生成（PNG 顶部存图像底部），与 UE 导入 OBJ 后的 mesh UV 方向一致，采样时直接按水体 UV 取即可吻合。

**方向计算（Python 端）**：
1. 对水体每个像素，找中心线最近点，取切线方向为基础流。
2. 找最近岸边点，取岸边切线（高斯模糊 + Sobel 梯度计算，平滑二值边界锯齿）。
3. 按到岸边距离加权混合：中心区域以中心线切线为主，靠近岸边时岸边切线影响增大。
4. 叠加低频分形 Perlin 噪声扰动，打破规律感。
5. 归一化后存 R/G。

**速度大小（Python 端）**：
- 以到岸边距离控制基础速度：河道中心快、岸边慢。
- 叠加低频噪声做轻微速度扰动，模拟水流不规则性。
- 最终 B 通道 = `基础速度 × (1 + 0.2 × 噪声)`，范围裁剪到 `[0, 1]`。

**速度大小（UE 端）**：
- Python 已输出方向和带扰动的速度标量。
- UE 中 `BaseSpeed` Scalar 参数控制整体流速。
- 最终速度 = `normalize(R,G) × BaseSpeed × B`。

**污染源**：完全独立，用户另行控制注入位置和颜色，速度场只负责"已注入的物质怎么被带走"。

### 9.2 水体扩散实现管线

```
Python 端
  │
  ├── 输入：天地图矢量图 → classify_vecw.py → labels.tif
  │
  ├── label_postprocess.py → labels_10x_no_boundary.tif（10x 水体掩膜）
  │
  └── gen_water_flow.py（与地形 OBJ、3DGS 语义地图并行）
        │
        ├── 提取 terrain_water_centerline.csv（河流中心线，UE DataTable 导入）
        ├── 生成 terrain_water_centerline_preview.png（人工校验）
        └── 生成 terrain_velocity_field.png（静态速度场纹理）

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
  │     扩散 Grid 的每个格子按水体 UV 采样速度场 Texture，
  │     用速度方向做 Semi-Lagrangian Advection，实现污染物随水流扩散。
  │
  └── 如需动态/更复杂河岸插值，可在 UE 中按 Spline 实时生成速度场 RT 替代静态纹理。
```

## 十、待办

- [ ] Stage 4：UE 语义层（材质系统、语义查询、区域过滤、交互编辑）
- [x] 水体污染扩散 Grid2D 与 UE 材质联动（基础速度场纹理已接入；中心线 CSV Y 偏移已修复）
- [x] 速度场重构：中心线切线 + 岸边切线加权 + 低频 Perlin 噪声，中心快、岸边慢
- [x] 污染源注入系统（位置、颜色，与速度场解耦）
- [x] 水体流场独立成模块（gen_water_flow.py），与地形 OBJ / 3DGS 语义地图并行生成
- [ ] 完整版 3DGS：从卫星图直出完整 3DGS 渲染点云（需补充三维信息输入）
- [ ] labels.tif 第三通道补入真实建筑高度（GBA 或阴影法）
- [ ] 多测试区域支持

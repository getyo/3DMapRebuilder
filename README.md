# 3DMapRebuilder

从对齐的天地图卫星影像、天地图矢量底图和高程数据生成 3DGS 语义地图。

测试区域：河北三河鲍邱河附近农村区，约 0.47×0.47 km（117.0991°E, 39.8349°N 为中心），面积约 0.22 km²。

## 数据源

| 数据 | 来源 | 规格 | 说明 |
|------|------|------|------|
| **天地图卫星图** | 天地图 WMTS `img_w` zoom 18 | ~0.46m/px, EPSG:4326 | 256×256 瓦片拼接为 GeoTIFF |
| **天地图矢量图** | 天地图 WMTS `vec_w` zoom 18 | ~0.46m/px, RGBA 瓦片 | 含语义颜色编码（水体/建筑/道路/地面） |
| **DEM 高程** | SRTM1 v3.0 (NASA) | 30m 级别（1 弧秒） | 三次立方插值升采样至卫星图分辨率 |

> **注意**：本仓库代码不包含输入数据的地理对齐和插值步骤。天地图瓦片下载、SRTM1 拼接、DEM 三次插值、卫星图与矢量图对齐等预处理需另行完成，当前直接使用已处理好的输入文件。且测试输入数据并不为全部数据源，只用了16个瓦片大小，大约0.22平方千米。

## 总体架构

四层结构，已完成前两层：

```
输入层（预处理，外部完成）
  │
  ├── 天地图卫星图 (satellite.tif)   ├── 天地图矢量图 (vec_raw.png)   └── 高程图 (dem.tif)
  │
  ▼
┌──────────────────────────────────────────────────┐
│ 第一层：基本语义分类器（已完成 ✓）                   │
│ classify_vecw.py :: VecClassifier                  │
│ 从 vec_raw.png 的 RGBA 颜色分类出语义标签           │
│ 输出：water.tif, building.tif, road.tif, labels.tif │
└──────────────────────────────────────────────────┘
  │
  ▼
┌──────────────────────────────────────────────────┐
│ 第二层：语义地图生成（已完成 ✓）                    │
│ gen_semantic.py :: SemanticMapBuilder             │
│ 从 labels.tif + dem.tif 生成 3DGS 语义点云 PLY    │
│ 输出：semanticMap.ply（含语义属性扩展）              │
└──────────────────────────────────────────────────┘
  │
  ▼
┌──────────────────────────────────────────────────┐
│ 第三层：渲染地图生成（规划中）                       │
│ 从卫星图直出 3DGS 渲染点云（含真实色彩）             │
└──────────────────────────────────────────────────┘
  │
  ▼
┌──────────────────────────────────────────────────┐
│ 第四层：语义功能层（规划中）                         │
│ 语义查询、区域过滤、交互编辑等上层应用               │
└──────────────────────────────────────────────────┘
```

## 当前数据流

```
vec_raw.png (RGBA 矢量底图)         satellite.tif (RGB 卫星图基准)    dem.tif (高程, 对齐)
       │                                        │                        │
       │  classify_vecw.py                       │                        │
       ▼                                        │                        │
  ┌──────────┐                                  │                        │
  │ 颜色分类  │  ← 7 种 RGBA 颜色常量 (±2 容差)    │                        │
  └────┬─────┘                                  │                        │
       │                                         │                        │
       ├──→ water.tif (二值掩膜, class_id=0)      │                        │
       ├──→ road.tif (二值掩膜, class_id=20)      │                        │
       ├──→ building.tif (二值掩膜, class_id=40)  │                        │
       └──→ labels.tif (3 通道语义标签) ────────────┼──────────── dem.tif ──┤
                │   Band 1: class_id              │                        │
                │   Band 2: road_level            │                        │
                │   Band 3: height(占位)          │                        │
                ▼                                                         │
          gen_semantic.py ←───────────────────────────────────────────────┘
                │
                │  · 像素→米坐标转换（影像中心原点）
                │  · 建筑高度分配（3~10m, 面积自适应）
                │  · 语义边界/建筑拐角检测
                │  · 建筑墙面垂直分段高斯
                │  · 语义颜色 → 球谐 DC 系数
                ▼
        semanticMap.ply
        （65 floats/vertex, 260 bytes/vertex）
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
| Band 2 (road_level) | 道路等级 | 40=高速, 80=国道, 120=省道, 160=小路 |
| Band 3 (height) | 相对高度（占位） | 120=建筑, 0=其他 |

**分类优先级**：水体 > 道路 > 建筑（形态闭合） > 地面 > 透明区域（默认地面）

**颜色常量**（天地图 vec_w zoom 18 的 RGBA 像素值）：

| 类别 | R | G | B |
|------|---|---|---|
| 水体 | 171 | 198 | 239 |
| 高速 | 186 | 160 | 241 |
| 国道 | 254 | 205 | 120 |
| 省道 | 254 | 235 | 130 |
| 小路 | 255 | 255 | 255 |
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
pip install numpy pillow rasterio scipy plyfile
python classify_vecw.py     # 生成语义标签
python gen_semantic.py      # 生成 3DGS PLY（含自动前置分类）
```

`gen_semantic.py` 的 `main()` 会自动先执行分类再建图。单独运行 `classify_vecw.py` 只执行分类。

## 输出

- `TestInput/SanHe/water.tif` — 水体二值掩膜
- `TestInput/SanHe/building.tif` — 建筑二值掩膜（形态闭合）
- `TestInput/SanHe/road.tif` — 道路二值掩膜（四级合并）
- `TestInput/SanHe/labels.tif` — 三通道语义标签
- `TestInput/SanHe/semanticMap.ply` — 3DGS 语义点云（SIBR / gsplat viewer 可直接查看）

## 待办

- [ ] 第三层：渲染地图生成（天地图卫星色直出 3DGS）
- [ ] 第四层：语义功能层（语义查询、过滤、交互）
- [ ] labels.tif 第三通道补入真实建筑高度（GBA 或阴影法）
- [ ] 多测试区域支持

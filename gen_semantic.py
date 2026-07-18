#!/usr/bin/env python3
"""
语义地图 3DGS PLY 生成器 — OOP 重构

架构:
  PLYFile          — 纯数据格式定义 (字段/字节偏移/类型/读写)
  SemanticMapBuilder — 全局单例, 持有所有参数 + build() 主流程
  main()            — 入口
"""

import numpy as np, rasterio, os, math, time
from scipy import ndimage
from scipy.ndimage import binary_erosion
from plyfile import PlyData
from classify_vecw import SANHE_LABEL_DIR, SANHE_SATE,SANHE_VEC,\
VecClassifier


# ═══════════════════════════════════════════════════════════════
# PLY 纯数据格式定义
# ═══════════════════════════════════════════════════════════════

class PLYFile:
    """
    标准 3DGS PLY binary_little_endian + 语义扩展属性
    
    每个顶点 260 字节 (65 × float32):
      bytes  0- 23: xyz + nx,ny,nz (6 floats)
      bytes 24- 35: f_dc_0..2 (3 floats)
      bytes 36-215: f_rest_0..44 (45 floats)
      bytes216-219: opacity (1 float)
      bytes220-231: scale_0..2 (3 floats)
      bytes232-247: rot_0..3 (4 floats)
      bytes248-251: semantic_class (float)
      bytes252-255: ground_height (float)
      bytes256-259: relative_height (float)
    """

    # ── 字段分类 ──
    FIELDS_STANDARD  = ['x','y','z','nx','ny','nz']
    FIELDS_SH_DC     = ['f_dc_0','f_dc_1','f_dc_2']
    FIELDS_SH_REST   = [f'f_rest_{i}' for i in range(45)]
    FIELDS_OPACITY   = ['opacity']
    FIELDS_SCALE     = ['scale_0','scale_1','scale_2']
    FIELDS_ROT       = ['rot_0','rot_1','rot_2','rot_3']
    FIELDS_SEMANTIC  = ['semantic_class','ground_height','relative_height']

    ALL_FIELDS = (FIELDS_STANDARD + FIELDS_SH_DC + FIELDS_SH_REST +
                  FIELDS_OPACITY + FIELDS_SCALE + FIELDS_ROT + FIELDS_SEMANTIC)

    # ── 字节布局 ──
    OFFSETS     = {name: i * 4 for i, name in enumerate(ALL_FIELDS)}
    BYTES_PER   = len(ALL_FIELDS) * 4   # 260

    # ── numpy dtype ──
    @classmethod
    def dtype(cls):
        return [(name, 'f4') for name in cls.ALL_FIELDS]

    # ── 读写 ──
    @classmethod
    def write_header(cls, f, n_verts: int):
        """写入 binary little-endian 头部"""
        f.write(b"ply\nformat binary_little_endian 1.0\n")
        f.write(f"element vertex {n_verts}\n".encode())
        for name in cls.ALL_FIELDS:
            f.write(f"property float {name}\n".encode())
        f.write(b"end_header\n")

    @classmethod
    def create_array(cls, n: int) -> np.ndarray:
        """创建空顶点数组"""
        return np.zeros(n, dtype=cls.dtype())

    @classmethod
    def read(cls, path: str):
        """读取 PLY 文件返回结构化数组"""
        
        return PlyData.read(path)['vertex'].data


# ═══════════════════════════════════════════════════════════════
# 语义地图构建器 (全局单例)
# ═══════════════════════════════════════════════════════════════

class SemanticMapBuilder:
    """
    从 DEM + 语义标签生成标准 3DGS PLY
    
    用法:
      b = SemanticMapBuilder()
      b.set_input(label_path='...', dem_path='...')
      b.build()
    
    或直接构造:
      b = SemanticMapBuilder(label_path='...', dem_path='...')
      b.build()
    """

    # ═══════════════════════════════════════════
    # 静态配置 — I/O 路径
    # ═══════════════════════════════════════════
    IN_LABEL_FILE  = 'TestInput/SanHe/labels.tif'
    IN_DEM_FILE = 'TestInput/SanHe/dem.tif'
    OUT_FILE = 'TestInput/SanHe/semanticMap.ply'

    # ═══════════════════════════════════════════
    # 静态配置 — 几何参数
    # ═══════════════════════════════════════════
    PX_M         = 0.458      # 像素地面尺寸 (m)
    BUILD_MIN_H  = 3.0        # 建筑最小高度 (m)
    BUILD_MAX_H  = 10.0       # 建筑最大高度 (m)
    M_DEG_LAT    = 111111.0   # 每度纬度 ≈ 米

    # ═══════════════════════════════════════════
    # 静态配置 — 高斯尺度 (sigma)
    # ═══════════════════════════════════════════
    SIGMA_H             = 0.7     # 内部水平 σ (3σ=2.1m 填充面)
    SIGMA_CONTOUR       = 0.001   # 轮廓 σ (屋顶垂直/地面边界/墙面拐角统一)
    SIGMA_V_GROUND      = 0.5     # 地面/水体/道路 垂直 σ
    WALL_STEP           = 0.5     # 墙面垂直分段间距 (m)
    WALL_THICK_NORMAL   = 0.5     # 墙面直边 法向 σ

    # ═══════════════════════════════════════════
    # 静态配置 — 渲染
    # ═══════════════════════════════════════════
    SH_C0       = 0.28209479177387814  # 球谐 DC 归一化系数
    OPACITY     = 0.95                 # 全局不透明度

    # ═══════════════════════════════════════════
    # 静态配置 — 颜色定义 (RGB [0,1])
    # ═══════════════════════════════════════════
    COLOR_WATER    = (100/255, 180/255, 255/255)  # 浅蓝
    COLOR_GROUND   = (180/255, 150/255, 100/255)  # 棕
    COLOR_BUILDING = (255/255, 240/255, 100/255)  # 黄
    COLOR_ROAD     = (180/255, 100/255, 255/255)  # 紫
    COLOR_WALL     = (255/255, 165/255,   0/255)  # 橙

    # 语义类别 → 颜色 映射
    CLASS_COLORS = {
        0: COLOR_WATER,     # 水
        1: COLOR_GROUND,    # 地
        2: COLOR_BUILDING,  # 建筑
        3: COLOR_ROAD,      # 路
    }

    # ═══════════════════════════════════════════
    # 辅助
    # ═══════════════════════════════════════════
    @staticmethod
    def _sq(s):
        """σ → log(σ), 3DGS 尺度编码"""
        return np.log(np.maximum(s, 1e-7))

    @staticmethod
    def _rgb_to_sh_dc(r, g, b):
        """RGB → 球谐 DC 系数"""
        return ((r - 0.5) / SemanticMapBuilder.SH_C0,
                (g - 0.5) / SemanticMapBuilder.SH_C0,
                (b - 0.5) / SemanticMapBuilder.SH_C0)

    # ═══════════════════════════════════════════
    # 实例初始化
    # ═══════════════════════════════════════════
    def __init__(self, label_path=None, dem_path=None, out_path=None):
        self.label_path = label_path or self.IN_LABEL_FILE
        self.dem_path   = dem_path   or self.IN_DEM_FILE
        self.out_path   = out_path   or self.OUT_FILE

        # 运行时数据
        self._class_id   = None
        self._dem        = None
        self._tf         = None
        self._height     = None
        self._width      = None
        self._bld_mask   = None
        self._bld_h      = None
        self._outline    = None
        self._corner     = None
        self._semantic   = None
        self._center_lon = None
        self._center_lat = None
        self._m_deg_lon  = None

    # ═══════════════════════════════════════════
    # 输入设置
    # ═══════════════════════════════════════════
    def set_input(self, label_path=None, dem_path=None, out_path=None):
        """设置或覆盖输入/输出路径"""
        if label_path: self.label_path = label_path
        if dem_path:   self.dem_path   = dem_path
        if out_path:   self.out_path   = out_path
        return self

    # ═══════════════════════════════════════════
    # 内部 — 加载
    # ═══════════════════════════════════════════
    def _load(self):
        with rasterio.open(self.dem_path) as d:
            self._dem = d.read(1).astype(np.float32)
        with rasterio.open(self.label_path) as l:
            self._class_id = l.read(1).astype(np.uint8)
            self._tf       = l.transform
            self._height   = l.height
            self._width    = l.width

    # ═══════════════════════════════════════════
    # 内部 — 坐标系
    # ═══════════════════════════════════════════
    def _setup_coords(self):
        """以影像物理中心为原点, 计算米/度转换系数"""
        self._center_lon = self._tf[2] + (self._width/2)*self._tf[0] + (self._height/2)*self._tf[1]
        self._center_lat = self._tf[5] + (self._width/2)*self._tf[3] + (self._height/2)*self._tf[4]
        self._m_deg_lon  = self.M_DEG_LAT * math.cos(math.radians(self._center_lat))

    # ═══════════════════════════════════════════
    # 内部 — 建筑高度
    # ═══════════════════════════════════════════
    def _build_heights(self):
        self._bld_mask = self._class_id == 40
        bld_labels, n = ndimage.label(self._bld_mask)
        self._bld_h = np.zeros_like(self._dem, dtype=np.float32)
        np.random.seed(42)
        for lbl in range(1, n+1):
            mask = bld_labels == lbl
            sz = mask.sum()
            if sz > 10:
                h = self.BUILD_MIN_H + (self.BUILD_MAX_H - self.BUILD_MIN_H) * (1 - math.exp(-sz/5000))
                h += np.random.uniform(-0.5, 0.5)
                h = np.clip(h, self.BUILD_MIN_H, self.BUILD_MAX_H)
            else:
                h = np.random.uniform(0, 0.5)
            self._bld_h[mask] = h

    # ═══════════════════════════════════════════
    # 内部 — 轮廓检测
    # ═══════════════════════════════════════════
    def _detect_edges(self):
        # 语义边界: 4邻域不同类
        U = np.roll(self._class_id, -1,0); U[-1,:]=0
        D = np.roll(self._class_id,  1,0); D[ 0,:]=0
        L = np.roll(self._class_id, -1,1); L[:,-1]=0
        R = np.roll(self._class_id,  1,1); R[:, 0]=0
        self._outline = (self._class_id != U) | (self._class_id != D) | (self._class_id != L) | (self._class_id != R)

        # 建筑拐角: 2方向同时暴露
        U_b = np.roll(self._bld_mask, -1,0); U_b[-1,:]=False
        D_b = np.roll(self._bld_mask,  1,0); D_b[ 0,:]=False
        L_b = np.roll(self._bld_mask, -1,1); L_b[:,-1]=False
        R_b = np.roll(self._bld_mask,  1,1); R_b[:, 0]=False
        n_exposed = ((~U_b).astype(np.int32) + (~D_b).astype(np.int32) +
                     (~L_b).astype(np.int32) + (~R_b).astype(np.int32))
        self._corner = self._bld_mask & (n_exposed >= 2)

    # ═══════════════════════════════════════════
    # 内部 — 语义映射
    # ═══════════════════════════════════════════
    def _build_semantic(self):
        self._semantic = np.zeros_like(self._class_id, dtype=np.float32)
        self._semantic[self._class_id == 0]  = 0   # water
        self._semantic[self._class_id == 60] = 1   # ground
        self._semantic[self._class_id == 40] = 2   # building
        self._semantic[self._class_id == 20] = 3   # road

    # ═══════════════════════════════════════════
    # 内部 — 像素 → 米坐标
    # ═══════════════════════════════════════════
    def _px_to_meters(self, r, c):
        lon = self._tf[2] + (c + 0.5)*self._tf[0] + (r + 0.5)*self._tf[1]
        lat = self._tf[5] + (c + 0.5)*self._tf[3] + (r + 0.5)*self._tf[4]
        x = (lon - self._center_lon) * self._m_deg_lon
        y = (lat - self._center_lat) * self.M_DEG_LAT
        return x, y

    # ═══════════════════════════════════════════
    # 内部 — 墙面高斯
    # ═══════════════════════════════════════════
    def _build_walls(self):
        bld_eroded = binary_erosion(self._bld_mask, structure=np.ones((3,3), dtype=bool))
        perimeter = self._bld_mask & ~bld_eroded
        pr, pc = np.where(perimeter)

        # 墙面颜色 SH DC
        sh_w = self._rgb_to_sh_dc(*self.COLOR_WALL)
        op_log = math.log(self.OPACITY / (1 - self.OPACITY))

        walls = []
        for i in range(len(pr)):
            r, c = int(pr[i]), int(pc[i])
            h = float(self._bld_h[r, c])
            if h <= 0.5: continue

            x_center, y_center = self._px_to_meters(r, c)
            elev = float(self._dem[r, c])
            n_wall = max(1, int(h / self.WALL_STEP))
            vert_step = h / n_wall
            sv = self._sq(vert_step * 0.4)
            is_corner = self._corner[r, c]
            wt = self.SIGMA_CONTOUR if is_corner else self.WALL_THICK_NORMAL

            for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
                nr, nc = r + dr, c + dc
                if nr < 0 or nr >= self._height or nc < 0 or nc >= self._width: continue
                if self._bld_mask[nr, nc]: continue

                if dr != 0:  # 南北面
                    ox, oz = 0.0, (-0.5*self.PX_M if dr < 0 else 0.5*self.PX_M)
                    s0, s2 = self._sq(self.SIGMA_H), self._sq(wt)
                else:        # 东西面
                    ox, oz = (-0.5*self.PX_M if dc < 0 else 0.5*self.PX_M), 0.0
                    s0, s2 = self._sq(wt), self._sq(self.SIGMA_H)

                for wi in range(n_wall):
                    wy = elev + (wi + 0.5) * vert_step
                    walls.append((x_center+ox, -wy, y_center+oz,
                                  *sh_w, s0, sv, s2, op_log, elev, h))
        return walls

    # ═══════════════════════════════════════════
    # 内部 — 屋顶/地面 (numpy 向量化)
    # ═══════════════════════════════════════════
    def _build_roof_ground(self):
        sH, sW = self._height, self._width
        rows, cols = np.arange(sH), np.arange(sW)
        rr, cc = np.meshgrid(rows, cols, indexing='ij')
        rr, cc = rr.ravel(), cc.ravel()
        n = len(rr)

        # 位置
        all_lon = self._tf[2] + (cc.astype(np.float32)+0.5)*self._tf[0] + (rr.astype(np.float32)+0.5)*self._tf[1]
        all_lat = self._tf[5] + (cc.astype(np.float32)+0.5)*self._tf[3] + (rr.astype(np.float32)+0.5)*self._tf[4]
        x_m = (all_lon - self._center_lon) * self._m_deg_lon
        y_m = (all_lat - self._center_lat) * self.M_DEG_LAT
        elev = self._dem[rr, cc]
        cid  = self._class_id[rr, cc]
        bh   = self._bld_h[rr, cc]
        sem  = self._semantic[rr, cc]

        is_bld = cid == 40
        z_m = np.where(is_bld, elev + bh, elev)

        # viewer: (east, -elev, north)
        vx = x_m
        vy = -z_m
        vz = y_m

        # 颜色 → SH DC
        sh = np.zeros((n, 3), dtype=np.float32)
        for sc_id in range(4):
            mask = sem == sc_id
            cr, cg, cb = self.CLASS_COLORS[sc_id]
            dc0, dc1, dc2 = self._rgb_to_sh_dc(cr, cg, cb)
            sh[mask, 0] = dc0
            sh[mask, 1] = dc1
            sh[mask, 2] = dc2

        # 轮廓自适应 σ
        is_out = self._outline[rr, cc]
        s_h = np.where(is_out, self._sq(self.SIGMA_CONTOUR), self._sq(self.SIGMA_H))
        sv  = np.where(is_bld, self.SIGMA_CONTOUR, self.SIGMA_V_GROUND)

        op_log = math.log(self.OPACITY / (1 - self.OPACITY))

        chunk = PLYFile.create_array(n)
        chunk['x'] = vx.astype(np.float32)
        chunk['y'] = vy.astype(np.float32)
        chunk['z'] = vz.astype(np.float32)
        chunk['f_dc_0'] = sh[:,0]
        chunk['f_dc_1'] = sh[:,1]
        chunk['f_dc_2'] = sh[:,2]
        chunk['opacity'] = op_log
        chunk['scale_0'] = s_h
        chunk['scale_1'] = self._sq(sv)
        chunk['scale_2'] = s_h
        chunk['rot_0'] = 1.0
        chunk['semantic_class'] = sem.astype(np.float32)
        chunk['ground_height']  = elev.astype(np.float32)
        chunk['relative_height'] = np.where(is_bld, bh, 0.0).astype(np.float32)

        return chunk

    # ═══════════════════════════════════════════
    # 内部 — 墙面数据写入
    # ═══════════════════════════════════════════
    @staticmethod
    def _walls_to_array(wall_data):
        n = len(wall_data)
        wa = PLYFile.create_array(n)
        for i, w in enumerate(wall_data):
            wa[i] = (
                w[0],w[1],w[2], 0,0,0,
                w[3],w[4],w[5],
                *([0.0]*45),
                w[9], w[6],w[7],w[8],
                1.0,0.0,0.0,0.0,
                2.0, w[10],w[11]
            )
        return wa

    # ═══════════════════════════════════════════
    # 主构建流程
    # ═══════════════════════════════════════════
    def build(self):
        t0 = time.time()
        os.makedirs(os.path.dirname(self.out_path), exist_ok=True)

        # 1. 加载
        self._load()
        print(f'Loaded: {self._width}x{self._height} px')

        # 2. 坐标系
        self._setup_coords()
        print(f'Center: ({self._center_lon:.6f}, {self._center_lat:.6f})')

        # 3. 建筑高度
        self._build_heights()
        print(f'Buildings: {self._bld_mask.sum():,} px, h=[{self._bld_h[self._bld_h>0].min():.1f}~{self._bld_h.max():.1f}]m')

        # 4. 轮廓检测
        self._detect_edges()
        print(f'Outline: {self._outline.sum():,} px | Corners: {self._corner.sum():,} px')

        # 5. 语义映射
        self._build_semantic()

        # 6. 墙面
        print('Walls (orange)...')
        wall_data = self._build_walls()
        nw = len(wall_data)
        print(f'  {nw:,} wall gaussians')

        # 7. 屋顶/地面
        chunk = self._build_roof_ground()
        n_rg = len(chunk)
        n_total = n_rg + nw
        print(f'Roof/ground: {n_rg:,} + walls: {nw:,} = {n_total:,}')

        # 8. 写出
        with open(self.out_path, 'wb') as f:
            PLYFile.write_header(f, n_total)
            f.write(chunk.tobytes())
            if nw > 0:
                wa = self._walls_to_array(wall_data)
                f.write(wa.tobytes())

        mb = os.path.getsize(self.out_path) / 1024 / 1024
        dt = time.time() - t0
        print(f'Done! {mb:.0f} MB, {dt:.0f}s')
        print(f'Params: sigma_h={self.SIGMA_H} contour={self.SIGMA_CONTOUR} wall_n={self.WALL_THICK_NORMAL}')
        print(f'File: {os.path.abspath(self.out_path)}')


# ═══════════════════════════════════════════════════════════════
# 入口
# ═══════════════════════════════════════════════════════════════

def main():
    classifier = VecClassifier()
    classifier.set_input(SANHE_VEC, SANHE_SATE)
    classifier.set_output(SANHE_LABEL_DIR)
    classifier.run()
    builder = SemanticMapBuilder(label_path= classifier.label_out)
    builder.build()


if __name__ == '__main__':
    main()

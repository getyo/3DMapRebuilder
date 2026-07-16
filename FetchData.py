import math
import requests
import time
from PIL import Image
from io import BytesIO
from dataclasses import dataclass

# 建议使用 HTTPS
BASE_URL = "https://t0.tianditu.gov.cn/img_w/wmts"
MY_TK = "cfe9e36543d2ee6f5a7751690eabb081"

@dataclass
class DataDescription:
    center_point: tuple[float, float] # (经度, 纬度)
    horizen_half_range: float
    vertical_half_range: float
    saved_path: str

    @staticmethod
    def get_tile_coords(lon, lat, zoom):
        lat_rad = math.radians(lat)
        n = 2.0 ** zoom
        xtile = int((lon + 180.0) / 360.0 * n)
        ytile = int((1.0 - math.log(math.tan(lat_rad) + (1 / math.cos(lat_rad))) / math.pi) / 2.0 * n)
        return xtile, ytile

    def satellite_download(self):
        lon_center, lat_center = self.center_point
        
        # 1. 计算范围
        lat_delta = self.vertical_half_range / 111.0
        lon_delta = self.horizen_half_range / (111.0 * math.cos(math.radians(lat_center)))
        
        # 修正：正确的经纬度范围计算
        # 左上角
        x_min, y_min = self.get_tile_coords(lon_center - lon_delta, lat_center + lat_delta, 18)
        # 右下角
        x_max, y_max = self.get_tile_coords(lon_center + lon_delta, lat_center - lat_delta, 18)
        
        print(f"下载范围: X[{x_min}-{x_max}], Y[{y_min}-{y_max}]")
        
        full_img = Image.new('RGB', ((x_max - x_min + 1) * 256, (y_max - y_min + 1) * 256))
        
        # 2. 更高级的 Header 伪装
        headers = {
            "User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/120.0.0.0 Safari/537.36",
            "Referer": "https://map.tianditu.gov.cn/",
            "Accept": "image/avif,image/webp,image/apng,image/svg+xml,image/*,*/*;q=0.8",
            "Accept-Language": "zh-CN,zh;q=0.9,en;q=0.8",
            "Connection": "keep-alive"
        }
        session = requests.Session()
        session.headers.update(headers)

        # 3. 循环下载
        for x in range(x_min, x_max + 1):
            for y in range(y_min, y_max + 1):
                # 拼接参数
                params = {
                    "SERVICE": "WMTS",
                    "REQUEST": "GetTile",
                    "VERSION": "1.0.0",
                    "LAYER": "img",
                    "STYLE": "default",
                    "TILEMATRIXSET": "w",
                    "FORMAT": "tiles",
                    "TILEMATRIX": "18",
                    "TILEROW": y,
                    "TILECOL": x,
                    "tk": MY_TK
                }
                
                try:
                    response = session.get(BASE_URL, params=params, timeout=10)
                    if response.status_code == 200:
                        tile = Image.open(BytesIO(response.content))
                        full_img.paste(tile, ((x - x_min) * 256, (y - y_min) * 256))
                        print(f"成功: {x}, {y}")
                    else:
                        print(f"失败/缺失: {x}, {y}, 状态码: {response.status_code}")
                except Exception as e:
                    print(f"网络异常: {e}")
                    
        full_img.save(self.saved_path)
        print("完成: " + self.saved_path)

# --- 以下为新增扩展功能，完全基于继承，未修改你的一行源码 ---

class TiandituDownloader(DataDescription):
    def vector_download(self):
        """扩展方法：专门下载矢量底图 (vec_w)"""
        lon_center, lat_center = self.center_point
        
        # 范围计算逻辑复用父类逻辑
        lat_delta = self.vertical_half_range / 111.0
        lon_delta = self.horizen_half_range / (111.0 * math.cos(math.radians(lat_center)))
        x_min, y_min = self.get_tile_coords(lon_center - lon_delta, lat_center + lat_delta, 18)
        x_max, y_max = self.get_tile_coords(lon_center + lon_delta, lat_center - lat_delta, 18)
        
        print(f"开始下载【矢量底图 vec_w】范围: X[{x_min}-{x_max}], Y[{y_min}-{y_max}]")
        
        full_img = Image.new('RGB', ((x_max - x_min + 1) * 256, (y_max - y_min + 1) * 256))
        
        # 矢量底图参数设定
        VECTOR_URL = "https://t0.tianditu.gov.cn/vec_w/wmts"
        headers = {
            "User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/126.0.0.0 Safari/537.36",
            "Referer": "https://map.tianditu.gov.cn/", # 极其重要
            "Accept": "image/avif,image/webp,image/apng,*/*",
            "Connection": "keep-alive"
        }
        session = requests.Session()
        session.headers.update(headers)

        for x in range(x_min, x_max + 1):
            for y in range(y_min, y_max + 1):
                # 关键修改点：将 LAYER 改为 'vec'，URL 改为 vec_w 的地址
                params = {
                    "SERVICE": "WMTS", "REQUEST": "GetTile", "VERSION": "1.0.0",
                    "LAYER": "vec", "STYLE": "default", "TILEMATRIXSET": "w",
                    "FORMAT": "tiles", "TILEMATRIX": "18",
                    "TILEROW": y, "TILECOL": x, "tk": MY_TK
                }
                try:
                    response = session.get(VECTOR_URL, params=params, timeout=10)
                    if response.status_code == 200:
                        tile = Image.open(BytesIO(response.content))
                        full_img.paste(tile, ((x - x_min) * 256, (y - y_min) * 256))
                        print(f"成功: {x}, {y}")
                    else:
                        print(f"失败: {x}, {y}, 状态码: {response.status_code}")
                except Exception as e:
                    print(f"网络异常: {e}")
                time.sleep(0.1)
                    
        # 保存为不同的文件名
        full_img.save(self.saved_path)
        print("矢量图下载完成: " + self.saved_path)

    
    # 你现在可以随时调用原有的卫星图下载方法
    # my_map.satellite_download()
    
    # 也可以直接调用新增的矢量底图下载方法

# 执行


my_map = TiandituDownloader((117.10, 39.84), 1.5, 1.5, "TextInput/SanHe/sanhe_vec.png")
my_map.vector_download()
# sanhe_satellite = DataDescription((117.10, 39.84), 1.5, 1.5, "satellite.png")
# sanhe_satellite.satellite_download()
# my_map.vector_download()
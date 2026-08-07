import os
import csv
import glob
import math

# 定义常量
WGS84_A = 6378137.0        # WGS84椭球体长半轴
WGS84_E = 0.0818191908425  # WGS84椭球体第一偏心率

def load_mine_origin(config_path, selected_name=None):
    """使用 Python 标准库读取项目约定的简单 mine_origins.yaml。"""
    required = {
        'GLOBAL_ORIGIN_LAT',
        'GLOBAL_ORIGIN_LON',
        'GLOBAL_ORIGIN_ALT',
    }
    origins = {}
    current_name = None

    with open(config_path, 'r', encoding='utf-8') as config_file:
        for line_number, raw_line in enumerate(config_file, 1):
            line = raw_line.split('#', 1)[0].rstrip()
            if not line.strip() or line.strip() == 'mine_origins:':
                continue

            indent = len(line) - len(line.lstrip(' '))
            key, separator, value = line.strip().partition(':')
            if not separator:
                raise ValueError(f'mine_origins.yaml 第 {line_number} 行格式错误')

            if indent == 2 and not value.strip():
                current_name = key.strip()
                origins[current_name] = {}
            elif indent == 4 and current_name is not None:
                field = key.strip()
                if field not in required:
                    raise ValueError(
                        f'mine_origins.yaml 第 {line_number} 行包含不支持的变量 {field}')
                origins[current_name][field] = float(value.strip())
            else:
                raise ValueError(f'mine_origins.yaml 第 {line_number} 行缩进错误')

    if not origins:
        raise ValueError('mine_origins.yaml 中没有矿山配置')
    mine_name = selected_name or next(iter(origins))
    if mine_name not in origins:
        raise ValueError(f'mine_origins.yaml 中不存在矿山 {mine_name}')
    missing = required - origins[mine_name].keys()
    if missing:
        raise ValueError(f'矿山 {mine_name} 缺少变量: {", ".join(sorted(missing))}')
    return mine_name, origins[mine_name]


CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           'mine_origins.yaml')
MINE_NAME, MINE_ORIGIN = load_mine_origin(
    CONFIG_PATH, os.environ.get('GEDITOR_MINE_NAME'))
GLOBAL_ORIGIN_LAT = MINE_ORIGIN['GLOBAL_ORIGIN_LAT']
GLOBAL_ORIGIN_LON = MINE_ORIGIN['GLOBAL_ORIGIN_LON']
GLOBAL_ORIGIN_ALT = MINE_ORIGIN['GLOBAL_ORIGIN_ALT']

class Point3d:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z
    
    def __str__(self):
        return f"Point3d(x={self.x}, y={self.y}, z={self.z})"

def trans_gps_to_pt(longitude, latitude, altitude):
    """
    从经纬高转换为局部平面坐标
    
    Args:
        longitude: 经度
        latitude: 纬度
        altitude: 高度
    
    Returns:
        Point3d: 局部平面坐标点
    """
    f_l0 = GLOBAL_ORIGIN_LAT
    f_lamda0 = GLOBAL_ORIGIN_LON
    f_hb = GLOBAL_ORIGIN_ALT
    
    f_re0 = WGS84_A / (math.sqrt(1.0 - WGS84_E * WGS84_E * math.sin(f_l0 * math.pi / 180) *
                                 math.sin(f_l0 * math.pi / 180.0)))
    f_x0 = (f_re0 + f_hb) * math.cos(f_l0 * math.pi / 180.0) * math.cos(f_lamda0 * math.pi / 180.0)
    f_y0 = (f_re0 + f_hb) * math.cos(f_l0 * math.pi / 180.0) * math.sin(f_lamda0 * math.pi / 180.0)
    f_z0 = ((1.0 - WGS84_E * WGS84_E) * f_re0 + f_hb) * math.sin(f_l0 * math.pi / 180.0)

    f_l = latitude
    f_lamda = longitude
    f_h = altitude
    
    f_re = WGS84_A / (math.sqrt(1.0 - WGS84_E * WGS84_E * math.sin(f_l * math.pi / 180.0) *
                                math.sin(f_l * math.pi / 180.0)))
    f_x = (f_re + f_h) * math.cos(f_l * math.pi / 180.0) * math.cos(f_lamda * math.pi / 180.0)
    f_y = (f_re + f_h) * math.cos(f_l * math.pi / 180.0) * math.sin(f_lamda * math.pi / 180.0)
    f_z = ((1.0 - WGS84_E * WGS84_E) * f_re + f_h) * math.sin(f_l * math.pi / 180.0)

    f_dx = f_x - f_x0
    f_dy = f_y - f_y0
    f_dz = f_z - f_z0
    
    f_dn = (-math.sin(f_l * math.pi / 180.0) * math.cos(f_lamda * math.pi / 180.0) * f_dx -
            math.sin(f_l * math.pi / 180.0) * math.sin(f_lamda * math.pi / 180.0) * f_dy +
            math.cos(f_l * math.pi / 180.0) * f_dz)
            
    f_de = (-math.sin(f_lamda * math.pi / 180.0) * f_dx + 
            math.cos(f_lamda * math.pi / 180.0) * f_dy)

    st_point3d = Point3d()
    st_point3d.x = f_de
    st_point3d.y = f_dn
    st_point3d.z = 0.0

    return st_point3d

def trans_pt_to_gps(point3d):
    """
    从局部平面坐标转换回经纬高
    
    Args:
        point3d: 局部平面坐标点
    
    Returns:
        Point3d: 经纬高坐标点
    """
    # 全局原点参数 - 将经纬度从角度转换为弧度
    f_l0 = GLOBAL_ORIGIN_LAT * math.pi / 180.0      # 全局原点纬度（弧度）
    f_lamda0 = GLOBAL_ORIGIN_LON * math.pi / 180.0  # 全局原点经度（弧度）
    f_hb = GLOBAL_ORIGIN_ALT                        # 全局原点高度

    # 计算全局原点的 ECEF 坐标
    f_re0 = WGS84_A / math.sqrt(1.0 - WGS84_E * WGS84_E * math.sin(f_l0) * math.sin(f_l0))
    f_x0 = (f_re0 + f_hb) * math.cos(f_l0) * math.cos(f_lamda0)
    f_y0 = (f_re0 + f_hb) * math.cos(f_l0) * math.sin(f_lamda0)
    f_z0 = ((1.0 - WGS84_E * WGS84_E) * f_re0 + f_hb) * math.sin(f_l0)

    # 局部平面坐标（ENU）到 ECEF 坐标的转换
    f_de = point3d.x  # 东向
    f_dn = point3d.y  # 北向
    f_du = point3d.z  # 天向

    # 计算目标点在 ECEF 坐标系中的坐标
    f_dx = (-math.sin(f_lamda0) * f_de - math.sin(f_l0) * math.cos(f_lamda0) * f_dn +
            math.cos(f_l0) * math.cos(f_lamda0) * f_du)
            
    f_dy = (math.cos(f_lamda0) * f_de - math.sin(f_l0) * math.sin(f_lamda0) * f_dn +
            math.cos(f_l0) * math.sin(f_lamda0) * f_du)
            
    f_dz = math.cos(f_l0) * f_dn + math.sin(f_l0) * f_du

    f_x = f_x0 + f_dx
    f_y = f_y0 + f_dy
    f_z = f_z0 + f_dz

    # 从 ECEF 坐标转换为经纬高
    f_p = math.sqrt(f_x * f_x + f_y * f_y)
    f_theta = math.atan2(f_z * WGS84_A, f_p * WGS84_A * (1.0 - WGS84_E * WGS84_E))

    f_latitude = math.atan2(f_z + WGS84_E * WGS84_E * WGS84_A * math.pow(math.sin(f_theta), 3),
                            f_p - WGS84_E * WGS84_E * WGS84_A * math.pow(math.cos(f_theta), 3))
                            
    f_longitude = math.atan2(f_y, f_x)
    
    f_re = WGS84_A / math.sqrt(1.0 - WGS84_E * WGS84_E * math.sin(f_latitude) * math.sin(f_latitude))
    f_altitude = f_p / math.cos(f_latitude) - f_re

    # 转换为度
    st_gps = Point3d()
    st_gps.y = f_latitude * 180.0 / math.pi
    st_gps.x = f_longitude * 180.0 / math.pi
    st_gps.z = f_altitude
    
    return st_gps

def latlon_to_utm(latitude, longitude):
    """
    将经纬度坐标转换为UTM坐标
    
    Args:
        latitude: 纬度
        longitude: 经度
    
    Returns:
        tuple: (utm_x, utm_y, zone_number, zone_letter)
    """
    # 计算UTM区域编号
    zone_number = int((longitude + 180) / 6) + 1
    
    # 计算UTM区域字母
    if -80 <= latitude <= 84:
        zone_letters = "CDEFGHJKLMNPQRSTUVWXX"
        zone_letter = zone_letters[int((latitude + 80) / 8)]
    else:
        zone_letter = 'X'  # 极地地区
    
    # 将经纬度转换为弧度
    lat_rad = math.radians(latitude)
    lon_rad = math.radians(longitude)
    lon0_rad = math.radians((zone_number - 1) * 6 - 180 + 3)  # 中央经线
    
    # WGS84椭球参数
    a = WGS84_A
    e = WGS84_E
    k0 = 0.9996  # 比例因子
    
    # 计算参数
    e_prime_sq = (e*e) / (1 - e*e)
    N = a / math.sqrt(1 - e*e * math.sin(lat_rad)**2)
    T = math.tan(lat_rad)**2
    C = e_prime_sq * math.cos(lat_rad)**2
    A = math.cos(lat_rad) * (lon_rad - lon0_rad)
    
    M = a * (
        (1 - e*e/4 - 3*e**4/64 - 5*e**6/256) * lat_rad -
        (3*e*e/8 + 3*e**4/32 + 45*e**6/1024) * math.sin(2*lat_rad) +
        (15*e**4/256 + 45*e**6/1024) * math.sin(4*lat_rad) -
        (35*e**6/3072) * math.sin(6*lat_rad)
    )
    
    # 计算UTM坐标
    utm_x = k0 * N * (
        A + (1-T+C) * A**3 / 6 +
        (5-18*T+T**2+72*C-58*e_prime_sq) * A**5 / 120
    )
    
    utm_y = k0 * (
        M + N * math.tan(lat_rad) * (
            A**2 / 2 +
            (5-T+9*C+4*C**2) * A**4 / 24 +
            (61-58*T+T**2+600*C-330*e_prime_sq) * A**6 / 720
        )
    )
    
    # 添加假东偏移（500000米）
    utm_x += 500000
    
    # 添加假北偏移（南半球为10000000米）
    if latitude < 0:
        utm_y += 10000000
    
    return (utm_x, utm_y, zone_number, zone_letter)

# # 原点坐标（保持原有的坐标偏移）
# ORIGIN_X = 612391.21
# ORIGIN_Y = 3445359.2

def process_segment_files(input_dir, input_mapping, output_dir, mapping_file):
    """
    处理segment_map目录下的所有CSV文件
    
    Args:
        input_dir: 输入目录路径
        input_mapping: 输入mapping文件路径
        output_dir: 输出目录路径
        mapping_file: mapping.txt文件路径
    """
    # 确保输出目录存在
    os.makedirs(output_dir, exist_ok=True)
    
    # 检查input_mapping文件是否存在，读取已有的映射关系
    existing_mapping = {}
    max_index = 0
    
    if os.path.exists(input_mapping):
        print(f"发现已有mapping文件: {input_mapping}，正在读取...")
        try:
            with open(input_mapping, 'r', newline='') as existing_map_f:
                existing_reader = csv.reader(existing_map_f)
                header = next(existing_reader, None)  # 跳过表头
                for row in existing_reader:
                    if len(row) >= 2:
                        filename = row[0]
                        index = int(row[1])
                        existing_mapping[filename] = index
                        max_index = max(max_index, index)
            print(f"成功读取已有映射关系，共{len(existing_mapping)}条记录，最大索引值: {max_index}")
        except Exception as e:
            print(f"读取已有mapping文件时出错: {e}，将重新创建映射关系")
            existing_mapping = {}
            max_index = 0
    else:
        print(f"警告: 输入mapping文件 {input_mapping} 不存在，将创建新的映射关系")
    
    # 获取所有CSV文件，并按文件名字母顺序排序
    csv_files = sorted(glob.glob(os.path.join(input_dir, "*.csv")))
    
    # 创建mapping文件
    with open(mapping_file, 'w', newline='') as map_f:
        map_writer = csv.writer(map_f)
        map_writer.writerow(['filename', 'index'])  # 写入表头，将iindex改为index
        
        # 处理每个CSV文件
        for file_path in csv_files:
            filename = os.path.basename(file_path)
            # 移除.csv后缀，再移除后四位字符
            filename_without_ext = os.path.splitext(filename)[0]
            if len(filename_without_ext) > 4:
                filename_without_ext = filename_without_ext[:-4]
            
            # 检查是否已有索引，若有则复用，若无则分配新索引
            if filename_without_ext in existing_mapping:
                index = existing_mapping[filename_without_ext]
                print(f"复用已有索引: {filename_without_ext} -> {index}")
            else:
                max_index += 1
                index = max_index
                print(f"分配新索引: {filename_without_ext} -> {index}")
            
            # 写入mapping文件，不包含.csv后缀
            map_writer.writerow([filename_without_ext, index])
            
            # 处理CSV文件，输出为.txt文件
            output_filename = filename_without_ext + '.txt'
            process_single_file(file_path, output_dir, output_filename, index, filename)
            
    print(f"处理完成，共处理 {len(csv_files)} 个文件")

def process_single_file(file_path, output_dir, output_filename, index, filename):
    """
    处理单个CSV文件
    
    Args:
        file_path: 输入文件路径
        output_dir: 输出目录路径
        output_filename: 输出文件名
        index: 文件对应的索引
        filename: 原始文件名
    """
    output_file = os.path.join(output_dir, output_filename)
    
    with open(file_path, 'r') as input_f, open(output_file, 'w', newline='') as output_f:
        reader = csv.reader(input_f)
        
        # 读取表头并将index列移到第一列
        header = next(reader)
        output_f.write(' '.join(['index'] + header) + '\n')
        
        # 存储所有数据行
        rows = list(reader)
        
        # # 处理所有数据行（去掉抽稀逻辑）
        # for row in rows:
        #     x = float(row[0]) #+ ORIGIN_X
        #     y = float(row[1]) #+ ORIGIN_Y
        #     # gps_pt = trans_pt_to_gps(Point3d(x, y, 0.0))
        #     (utm_x, utm_y, zone_number, zone_letter) = latlon_to_utm(y, x)
        #     # 更新坐标并将index移到第一列
        #     new_row = [str(index)] + [str(utm_x), str(utm_y)] + row[2:]
        #     output_f.write(' '.join(new_row) + '\n')

        # 抽稀处理：每5个点保留一个点，但保留首尾点
        if len(rows) <= 2:
            # 如果点数很少，保留所有点
            for row in rows:
                x = float(row[1]) #+ ORIGIN_X   lon
                y = float(row[0]) #+ ORIGIN_Y   lat
                # gps_pt = trans_pt_to_gps(Point3d(x, y, 0.0))
                (utm_x, utm_y, zone_number, zone_letter) = latlon_to_utm(y, x)
                # 更新坐标并将index移到第一列
                new_row = [str(index)] + [str(utm_x), str(utm_y)] + row[2:]
                output_f.write(' '.join(new_row) + '\n')

        else:
            # 保留第一点
            first_row = rows[0]
            x = float(first_row[0]) #+ ORIGIN_X
            y = float(first_row[1]) #+ ORIGIN_Y
            # gps_pt = trans_pt_to_gps(Point3d(x, y, 0.0))
            (utm_x, utm_y, zone_number, zone_letter) = latlon_to_utm(y, x)
            new_row = [str(index)] + [str(utm_x), str(utm_y)] + first_row[2:]
            output_f.write(' '.join(new_row) + '\n')
            # print(f"utm_x={utm_x}, utm_y={utm_y}")
            # print(f"x+ori={x+612391.21}, y+ori={y+3445359.2}")
            
            # 抽稀中间点：每5个点保留一个
            middle_rows = rows[1:-1]
            for i in range(0, len(middle_rows), 5):
                row = middle_rows[i]
                x = float(row[0]) #+ ORIGIN_X
                y = float(row[1]) #+ ORIGIN_Y
                # gps_pt = trans_pt_to_gps(Point3d(x, y, 0.0))
                (utm_x, utm_y, zone_number, zone_letter) = latlon_to_utm(y, x)
                new_row = [str(index)] + [str(utm_x), str(utm_y)] + row[2:]
                output_f.write(' '.join(new_row) + '\n')
            
            # 保留最后一点
            last_row = rows[-1]
            x = float(last_row[0]) #+ ORIGIN_X
            y = float(last_row[1]) #+ ORIGIN_Y
            # gps_pt = trans_pt_to_gps(Point3d(x, y, 0.0))
            (utm_x, utm_y, zone_number, zone_letter) = latlon_to_utm(y, x)
            new_row = [str(index)] + [str(utm_x), str(utm_y)] + last_row[2:]
            output_f.write(' '.join(new_row) + '\n')

if __name__ == "__main__":
    # 添加测试代码来验证trans_gps_to_pt函数
    # print("Testing trans_gps_to_pt function:")
    # origin_pt = trans_gps_to_pt(GLOBAL_ORIGIN_LON, GLOBAL_ORIGIN_LAT, 0.0)
    # print(f"Input: lon={GLOBAL_ORIGIN_LON}, lat={GLOBAL_ORIGIN_LAT}, alt=0.0")
    # print(f"Output: x={origin_pt.x}, y={origin_pt.y}, z={origin_pt.z}")
    # (utm_x, utm_y, zone_number, zone_letter) = latlon_to_utm(GLOBAL_ORIGIN_LAT, GLOBAL_ORIGIN_LON)
    # print(f"utm_x={utm_x}, utm_y={utm_y}")
    # exit(0)
    
    input_directory = "/home/project/HDMap_data/liquan/LQ_RTK_2026.06.30_V38_beta/Rtk_Map/gps"
    output_directory = "/home/project/HDMap_data/liquan/tra/20260716"
    input_mapping = "/home/project/HDMap_data/liquan/tra/20260716/mapping.txt"
    mapping_file = os.path.join(output_directory, "mapping.txt")
    
    process_segment_files(input_directory, input_mapping, output_directory, mapping_file)

import time
import pyproj
import math


def update_coordinate(sog, cog, time_interval, initial_x, initial_y):
    """
    按时间生成单条目标船的位置坐标
    :param sog: 船舶航速
    :param cog: 船舶航向
    :param time_interval: 坐标点更新时间间隔
    :param initial_x: 目标船初始x
    :param initial_y: 目标船初始y
    :return: 每一秒更新的位置坐标
    """
    x = initial_x
    y = initial_y

    while True:
        x += sog * math.sin(math.radians(cog))
        y += sog * math.cos(math.radians(cog))

        yield (x, y)

        time.sleep(time_interval)


def generate_ship_coordinates(ship_data):
    """
    一次更新多艘目标船的位置信息
    :param ship_data: 定义多个目标船状态信息
    :return: 多个目标船更新后的位置坐标
    """
    ships = []
    for data in ship_data:
        sog = data['sog']
        cog = data['cog']
        time_interval = data['time_interval']
        initial_x = data['initial_x']
        initial_y = data['initial_y']
        generator = update_coordinate(sog, cog, time_interval, initial_x, initial_y)
        ships.append(generator)

    while True:
        positions = []
        for generator in ships:
            position = next(generator)
            positions.append(position)
        yield positions


def convert_latlon_to_xy(lat, lon, src_epsg, dst_epsg):
    """
    船舶经纬度转（x，y）坐标：大地坐标系
    :param lat:
    :param lon:
    :param src_epsg:
    :param dst_epsg:
    :return: （x，y）坐标
    """
    proj = pyproj.Transformer.from_crs(src_epsg, dst_epsg, always_xy=True)

    x, y = proj.transform(lon, lat)  # 经度，纬度的顺序

    return x, y


def target_ship(lat_os_start, lon_os_start):
    """
    生成多条目标船的状态信息
    :param lat_os_start: 本船初始纬度
    :param lon_os_start: 本船初始经度
    :return: 多条目标船的状态信息,x,y坐标,航速航向
    """
    source_epsg = 'EPSG:4326'  # WGS84经纬度坐标系
    target_epsg = 'EPSG:3857'  # 地理坐标参考系统
    # ================================================================================================
    # 设置目标船经纬度，航速航向
    lat_ts = [lat_os_start + 0.02, lat_os_start - 0.02, lat_os_start + 0.01, lat_os_start]
    lon_ts = [lon_os_start, lon_os_start - 0.02, lon_os_start - 0.01, lon_os_start + 0.02]
    cog_ts = [180, 45, 135, 90]
    sog_ts = [12, 15, 12, 8]
    # ================================================================================================
    # 目标船经纬度转x，y坐标
    x_ts = []
    y_ts = []
    for i in range(len(lon_ts)):
        x, y = convert_latlon_to_xy(lat_ts[i], lon_ts[i], source_epsg, target_epsg)
        x_ts.append(x)
        y_ts.append(y)
    # ================================================================================================
    # 将对应的目标船信息分配到目标船，目标船根据时间更新位置
    ship_data = [
        {
            'sog': sog_ts[0],
            'cog': cog_ts[0],
            'time_interval': 1,
            'initial_x': x_ts[0],
            'initial_y': y_ts[0]
        },
        {
            'sog': sog_ts[1],
            'cog': cog_ts[1],
            'time_interval': 1,
            'initial_x': x_ts[1],
            'initial_y': y_ts[1]
        },
        {
            'sog': sog_ts[2],
            'cog': cog_ts[2],
            'time_interval': 1,
            'initial_x': x_ts[2],
            'initial_y': y_ts[2]
        },
        {
            'sog': sog_ts[3],
            'cog': cog_ts[3],
            'time_interval': 1,
            'initial_x': x_ts[3],
            'initial_y': y_ts[3]
        }
    ]
    coordinates_generator = generate_ship_coordinates(ship_data)
    while True:
        x_ts = []
        y_ts = []
        positions = next(coordinates_generator)
        for i in range(len(positions)):
            x_ts.append(positions[i][0])
            y_ts.append(positions[i][1])
        yield x_ts, y_ts, cog_ts, sog_ts


# if __name__ == "__main__":
#     lat_os_start, lon_os_start = start_own_ship()
#     ship_generator = target_ship(lat_os_start, lon_os_start)
#     while True:
#         x_ts, y_ts = next(ship_generator)
#         print(x_ts, y_ts)




# import ip_guidance
#
#
# Tug = [0, 0, 60, 5]
# track_point = [10, 10]
# h = 10
# heading_cmd, speed_cmd = ip_guidance.Point_track().solve(Tug, track_point, h)
# print(heading_cmd, speed_cmd)

import math
import time


def update_coordinate(sog, cog, time_interval, initial_x, initial_y):
    x = initial_x
    y = initial_y

    while True:
        x += sog * math.cos(math.radians(cog))
        y += sog * math.sin(math.radians(cog))

        yield (x, y)

        time.sleep(time_interval)


def generate_ship_coordinates(ship_data):
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


# 测试数据
ship_data = [
    {
        'sog': 10,
        'cog': 45,
        'time_interval': 1,
        'initial_x': 0,
        'initial_y': 0
    },
    {
        'sog': 8,
        'cog': 90,
        'time_interval': 1,
        'initial_x': 10,
        'initial_y': 5
    }
]

# 生成坐标数据
coordinates_generator = generate_ship_coordinates(ship_data)

# 控制测试时间，例如输出 10 个时间点的坐标
while True:
    positions = next(coordinates_generator)
    print(positions)

import numpy as np
import math


# 计算路径的长度
def cal_path_length(x, y):
    # 初始化总长度
    length = 0.0
    # 计算相邻点之间的距离并相加
    for i in range(1, len(x)):
        dx = x[i] - x[i - 1]
        dy = y[i] - y[i - 1]
        segment_length = np.sqrt(dx ** 2 + dy ** 2)
        length += segment_length
    return length


def select_elements(arr, num):
    n = len(arr)

    # 计算每个分片的长度
    slice_length = n // (num - 1)

    # 选取10个数值，按照当前顺序编号进行等分
    selected_values = [arr[i * slice_length] for i in range(num - 1)]
    selected_values.append(arr[-1])

    return selected_values


def cal_distance(x1, y1, x2, y2):
    # 使用欧几里得距离公式计算距离
    distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return distance


def cal_curvature(x, y):
    # 计算曲线的一阶和二阶导数
    dx = np.gradient(x)
    dy = np.gradient(y)
    d2x = np.gradient(dx)
    d2y = np.gradient(dy)
    # 计算曲率
    curvature = np.abs(dx * d2y - dy * d2x) / (dx ** 2 + dy ** 2) ** 1.5
    return curvature


def main():
    x = [1.0, 2.0, 3.0, 4.0, 5.0]
    y = [1.0, 2.0, 4.0, 7.0, 9.0]

    # 计算曲线长度
    curve_length = cal_path_length(x, y)
    distance = cal_distance(x[1], y[1], x[2], y[2])
    curvature = cal_curvature(x, y)
    print(f"The length of the curve is {curve_length:.2f}")
    print(distance)
    print(curvature)


if __name__ == "__main__":
    main()

import math


# 顺时针旋转
def rotate_points(point_list, psi):
    # 将角度转换成弧度
    psi = math.radians(psi)

    # 创建一个空的二维数组用于存储旋转后的坐标点
    rotated_points = []

    # 遍历二维数组中的每个坐标点
    for point in point_list:
        x, y = point
        new_x = round(x * math.cos(psi) + y * math.sin(psi), 5)
        new_y = round(-x * math.sin(psi) + y * math.cos(psi), 5)
        rotated_points.append([new_x, new_y])

    return rotated_points

if __name__ == "__main__":

    # 示例
    point_list = [
        [-1, 1],
        [-2, 4],
        [-3, 9],
        [-4, 16],
        [1, 1],
        [2, 4],
        [3, 9],
        [4, 16]
    ]

    # 指定旋转角度
    psi = 45

    # 调用函数进行旋转
    re_point_list = rotate_points(point_list, psi)

    # 打印旋转后的坐标点
    for point in re_point_list:
        print(point)
    print(re_point_list)






    # 画出来看看效果= =
    import matplotlib.pyplot as plt

    original_x = [point[0] for point in point_list]
    original_y = [point[1] for point in point_list]

    plt.scatter(original_x, original_y, label='point_list', color='blue')

    # 绘制旋转后的坐标点
    rotated_x = [point[0] for point in re_point_list]
    rotated_y = [point[1] for point in re_point_list]

    plt.scatter(rotated_x, rotated_y, label='re_point_list', color='red')

    # 添加图例
    plt.legend()

    # 设置坐标轴范围
    plt.xlim(-20, 20)
    plt.ylim(-20, 20)

    # 添加坐标轴标签
    plt.xlabel('X')
    plt.ylabel('Y')

    # 显示图形
    plt.show()
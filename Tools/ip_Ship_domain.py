import numpy as np
import matplotlib.pyplot as plt
import math


def quaternion_ship_domain(L, V, heading):
    # 船舶方向向量
    heading_rad = math.radians(heading)  # 将航向转换为弧度

    k_ad = 10 ** (0.3591 * math.log10(V) + 0.0952)
    k_dt = 10 ** (0.5441 * math.log10(V) - 0.0795)

    R_fore = (1 + 1.34 * math.sqrt(k_ad ** 2 + (k_ad / 2) ** 2)) * L
    R_aft = (1 + 0.67 * math.sqrt(k_ad ** 2 + (k_ad / 2) ** 2)) * L
    R_starb = (0.2 + k_dt) * L
    R_port = (0.2 + 0.75 * k_dt) * L

    # 计算船舶领域的点
    # 第1象限
    a1 = R_starb
    b1 = R_fore
    t = np.arange(0, 91)
    x1 = a1 * np.cos(np.deg2rad(t))  # 正常生成 x 坐标
    y1 = b1 * np.sin(np.deg2rad(t))  # 正常生成 y 坐标

    # 第2象限
    a2 = R_port
    b2 = R_fore
    t = np.arange(90, 181)
    x2 = a2 * np.cos(np.deg2rad(t))
    y2 = b2 * np.sin(np.deg2rad(t))

    # 第3象限
    a3 = R_port
    b3 = R_aft
    t = np.arange(180, 271)
    x3 = a3 * np.cos(np.deg2rad(t))
    y3 = b3 * np.sin(np.deg2rad(t))

    # 第4象限
    a4 = R_starb
    b4 = R_aft
    t = np.arange(270, 361)
    x4 = a4 * np.cos(np.deg2rad(t))
    y4 = b4 * np.sin(np.deg2rad(t))

    # 合并所有点
    x_all_rotated = np.concatenate((x1, x2, x3, x4))
    y_all_rotated = np.concatenate((y1, y2, y3, y4))

    return x_all_rotated, y_all_rotated


def rotate_points(x, y, angle):
    """ 旋转点 (x, y) 使用给定的角度 (以度为单位) """
    angle_rad = np.radians(-angle)  # 逆时针旋转
    rotation_matrix = np.array([[np.cos(angle_rad), -np.sin(angle_rad)],
                                [np.sin(angle_rad), np.cos(angle_rad)]])

    points = np.vstack((x, y))
    rotated_points = rotation_matrix @ points
    return rotated_points[0], rotated_points[1]


def plot_ship_domain(L, V, heading, position):
    # 生成船舶领域的点
    x_domain, y_domain = quaternion_ship_domain(L, V, heading)

    # 旋转船舶领域使其与航向一致
    x_domain, y_domain = rotate_points(x_domain, y_domain, heading)

    # 位置偏移
    x_domain += position[0]
    y_domain += position[1]
    x_domain = [element / 1852 for element in x_domain]
    y_domain = [element / 1852 for element in y_domain]
    position = [element / 1852 for element in position]

    # 绘制船舶领域
    plt.figure(figsize=(8, 8))
    plt.plot(x_domain, y_domain, linewidth=2)
    plt.scatter(position[0], position[1], color='red', marker='o', label='Ship Position')

    # 设置图形
    plt.xlim(-6, 6)
    plt.ylim(-6, 6)
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Ship Domain Visualization')
    plt.axhline(0, color='grey', linewidth=0.6, ls='--')
    plt.axvline(0, color='grey', linewidth=0.6, ls='--')
    plt.grid()
    plt.gca().set_aspect('equal', adjustable='box')
    plt.legend()
    plt.show()


# 示例参数
L = 160
V = 10
heading = 45  # 航向，0度为正北
position = (20, 20)  # 随机位置

# 调用绘图函数
plot_ship_domain(L, V, heading, position)

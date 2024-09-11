import numpy as np
import math
from shapely.geometry import Polygon, Point

ferry_route_origin = np.array([[793.46, 135.47], [2275.13, 5213.71], [2747.94, 5941.12]])
ferry_route = np.array([[810, 150], [2275, 5210], [2750, 5920]])
##固定航线航路点
route0 = np.array([[1050, 1010], [2240, 1010]])  # ferry_route 直行


route1 = np.array([[0, 630], [3000, 630]])
route2 = np.array([[280, 1600], [3000, 1245]])
route3 = np.array([[0, 3300], [650, 2400], [650, 0]])
route4 = np.array([[0, 3900], [1030, 2600], [3000, 1715]])
route5 = np.array([[3000, 2280], [1325, 3050], [0, 4770]])
route6 = np.array([[3000, 2915], [0, 6000]])
route7 = np.array([[3000, 3500], [1400, 6000]])
route8 = np.array([[3000, 1100], [0, 800]])
route9 = np.array([[0, 1000], [3000, 2700]])
route10 = np.array([[0, 3600], [2100, 800]])
route11 = np.array([[1200, 6000], [2500, 2850]])

###航线方向向量###
direction_route0 = np.array(route0[1]) - np.array(route0[0])
direction_route1 = np.array(route1[1]) - np.array(route1[0])
direction_route2 = np.array(route2[1]) - np.array(route2[0])
direction_route3_1 = np.array(route3[1]) - np.array(route3[0])
direction_route3_2 = np.array(route3[2]) - np.array(route3[1])
direction_route4_1 = np.array(route4[1]) - np.array(route4[0])
direction_route4_2 = np.array(route4[2]) - np.array(route4[1])
direction_route5_1 = np.array(route5[1]) - np.array(route5[0])
direction_route5_2 = np.array(route5[2]) - np.array(route5[1])
direction_route6 = np.array(route6[1]) - np.array(route6[0])
direction_route7 = np.array(route7[1]) - np.array(route7[0])
direction_route8 = np.array(route8[1]) - np.array(route8[0])
direction_route9 = np.array(route9[1]) - np.array(route9[0])
direction_route10 = np.array(route10[1]) - np.array(route10[0])
direction_route11 = np.array(route11[1]) - np.array(route11[0])

dv0 = np.array(direction_route0) / np.linalg.norm(direction_route0)
dv1 = np.array(direction_route1) / np.linalg.norm(direction_route1)
dv2 = np.array(direction_route2) / np.linalg.norm(direction_route2)
dv3_1 = np.array(direction_route3_1) / np.linalg.norm(direction_route3_1)
dv3_2 = np.array(direction_route3_2) / np.linalg.norm(direction_route3_2)
dv4_1 = np.array(direction_route4_1) / np.linalg.norm(direction_route4_1)
dv4_2 = np.array(direction_route4_2) / np.linalg.norm(direction_route4_2)
dv5_1 = np.array(direction_route5_1) / np.linalg.norm(direction_route5_1)
dv5_2 = np.array(direction_route5_2) / np.linalg.norm(direction_route5_2)
dv6 = np.array(direction_route6) / np.linalg.norm(direction_route6)
dv7 = np.array(direction_route7) / np.linalg.norm(direction_route7)
dv8 = np.array(direction_route8) / np.linalg.norm(direction_route8)
dv9 = np.array(direction_route9) / np.linalg.norm(direction_route9)
dv10 = np.array(direction_route10) / np.linalg.norm(direction_route10)
dv11 = np.array(direction_route11) / np.linalg.norm(direction_route11)


###船舶###
class Ship(object):
    def __init__(self, L, B, V):
        self.L = L  # m
        self.B = B  # m
        self.V = V  # m/s


###船舶类型###
# envFerry = envShip(75, 35, 4)  # (90,16,4.5) ; (52,15,3.8) ; (73,14,4)
# envContainer = envShip(224, 35, 4)  # (92,16,4) ;(79,16,3.5) ; (118,22,5)
# envBulk = envShip(190, 32, 3.9)  # (143,24,6);(56,10,3);(80,14,4.2)
# envTanker = envShip(147, 24, 5.5)  # (87，15,4.7);(135,22,6.3);(60,12,4.4)
# envOtherShip = envShip(58, 13, 4.76)  # (85，15,3.3);(48,10,3.9);(110,19,5.3)
def convert_angle_to_direction_vector(angle_deg):
    # 将角度转换为弧度
    angle_rad = math.radians(angle_deg)
    # 计算方向向量的 x 和 y 分量
    x = math.cos(angle_rad)
    y = math.sin(angle_rad)

    # 计算方向向量的长度
    length = math.sqrt(x ** 2 + y ** 2)

    # 归一化方向向量
    unit_vector = (x / length, y / length)

    return unit_vector
    # # 计算方向向量的 x 和 y 分量
    # return math.cos(angle_rad),math.sin(angle_rad)
#  用于计算船舶尾部两个点的坐标


def calculate_base_coordinates(A1, A2, H, angle):
    # 计算底边的长度
    base_length = (A2[0] - A1[0]) / math.cos(math.radians(angle))

    # 计算底边的中点坐标
    base_midpoint_x = (A1[0] + A2[0]) / 2
    base_midpoint_y = (A1[1] + A2[1]) / 2

    # 计算底边的半长
    base_half_length = base_length / 2

    # 计算底边的斜边与 x 轴的夹角
    base_angle = math.atan2(A2[1] - A1[1], A2[0] - A1[0])

    # 计算底边的两个顶点坐标
    B1_x = base_midpoint_x - base_half_length * math.cos(base_angle + math.radians(90))
    B1_y = base_midpoint_y - base_half_length * math.sin(base_angle + math.radians(90))
    B2_x = base_midpoint_x + base_half_length * math.cos(base_angle + math.radians(90))
    B2_y = base_midpoint_y + base_half_length * math.sin(base_angle + math.radians(90))

    return (B1_x, B1_y), (B2_x, B2_y)


#  用于计算船舶的三个关键点的坐标
def getShipBox(Ship, dv, current_pos):  # 船舶参数，速度，当前位置
    fp = np.array([0.5 * Ship.L * dv[0], 0.5 * Ship.L * dv[1]])
    bow_p = np.array([0.4 * Ship.L * dv[0], 0.4 * Ship.L * dv[1]])
    lp = np.array([-0.5 * Ship.B * dv[1], 0.5 * Ship.B * dv[0]])

    #### hull_box：实尺寸矩形轮廓
    lf_box = fp + lp + np.array(current_pos)
    lb_box = -fp + lp + np.array(current_pos)
    rb_box = -fp - lp + np.array(current_pos)
    rf_box = fp - lp + np.array(current_pos)
    hull_box = np.array([lf_box, lb_box, rb_box, rf_box])

    #### hull_shape：实尺寸船型轮廓
    f_shape = fp + np.array(current_pos)
    lb_shape = bow_p + lp + np.array(current_pos)
    rb_shape = bow_p - lp + np.array(current_pos)
    hull_shape = np.array([f_shape, lb_shape, lb_box, rb_box, rb_shape])

    #### hull_box_out：膨胀后的矩形轮廓
    f_out = np.array([0.5 * (Ship.L + 20) * dv[0], 0.5 * (Ship.L + 20) * dv[1]])
    b_out = np.array([-0.5 * (Ship.L) * dv[0], -0.5 * (Ship.L) * dv[1]])
    # bow_p_out = np.array([0.4 * (Ship.L+20) * dv[0], 0.4 * (Ship.L+20) * dv[1]])
    l_out = np.array([-0.5 * (Ship.B) * dv[1], 0.5 * (Ship.B) * dv[0]])
    r_out = np.array([0.5 * (Ship.B + 5) * dv[1], -0.5 * (Ship.B + 5) * dv[0]])

    lf_box_out = f_out + l_out + np.array(current_pos)
    lb_box_out = b_out + l_out + np.array(current_pos)
    rb_box_out = b_out + r_out + np.array(current_pos)
    rf_box_out = f_out + r_out + np.array(current_pos)
    hull_box_out = np.array([lf_box_out, lb_box_out, rb_box_out, rf_box_out])

    return hull_box_out, hull_shape


# 用于生成一个矩形区域内的点坐标
def getBoxObs(hull_box, resolution):  # hull_box,分辨率
    polygon = Polygon(hull_box)
    bounds = list(
        polygon.bounds)  # 这行代码获取多边形的边界框（bounding box），并将其转换为列表形式。边界框是一个矩形，由最小的 x 坐标、最小的 y 坐标、最大的 x 坐标和最大的 y 坐标组成。
    bounds[0] = math.floor(bounds[0] / resolution)
    bounds[1] = math.floor(bounds[1] / resolution)
    bounds[2] = math.ceil(bounds[2] / resolution)
    bounds[3] = math.ceil(bounds[3] / resolution)
    x_range = np.arange(bounds[0], bounds[2])
    y_range = np.arange(bounds[1], bounds[3])
    box_obs_index = []
    box_obs_coors = []
    for i in x_range:
        for j in y_range:
            index = [i, j]
            coors = [i * 30 + 15, j * 30 + 15]
            box_obs_index.append(index)
            box_obs_coors.append(coors)
    return box_obs_index, box_obs_coors



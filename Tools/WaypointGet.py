import numpy as np
import math
import socket
from Tools import TargetShipSet


"""计算DCPA/TCPA"""


def calculate_DCPA_TCPA(ship1, ship2):
    """
    计算给定两艘船舶之间的 DCPA (Distance of closest point of approach) 和 TCPA (Time of closest point of approach)。

    Args:
        ship1 (tuple): 包含船舶1的位置和速度信息，为一个元组 (x1, y1, v1, heading1)。
        ship2 (tuple): 包含船舶2的位置和速度信息，为一个元组 (x2, y2, v2, heading2)。

    Returns:
        tuple: 一个包含 DCPA 和 TCPA 值的元组 (dcpa, tcpa)。

    """

    # 解包船舶1的信息
    x1, y1, v1, heading1 = ship1

    # 解包船舶2的信息
    x2, y2, v2, heading2 = ship2

    # 计算船舶1和船舶2之间的位置矢量
    dx = x2 - x1
    dy = y2 - y1

    # 计算船舶1和船舶2之间的速度矢量
    vdx = v2 * math.sin(math.radians(heading2)) - v1 * math.sin(math.radians(heading1))
    vdy = v2 * math.cos(math.radians(heading2)) - v1 * math.cos(math.radians(heading1))

    # 计算 DCPA 和 TCPA 的值
    a = vdx**2 + vdy**2
    if math.isclose(a, 0.0):
        # 当两艘船舶的速度相同时，将 DCPA 和 TCPA 置为无穷大
        dcpa = float('inf')
        tcpa = float('inf')
        return (dcpa, tcpa)

    b = vdx*dx + vdy*dy
    c = dx**2 + dy**2

    t = -b / a
    if t < 0:
        # 如果 TCPA 已经过去，则将 DCPA 和 TCPA 置为无穷大
        dcpa = float('inf')
        tcpa = float('inf')
        return (dcpa, tcpa)

    dcpa_sq = c - b**2 / a
    if dcpa_sq < 0:
        # 如果 DCPA 是负数，则船舶已经相撞了
        dcpa = 0
        tcpa = t
        return (dcpa, tcpa)

    # 计算 DCPA 和 TCPA 的值
    dcpa = math.sqrt(dcpa_sq)
    tcpa = t
    return (dcpa, tcpa)


"""船舶领域半径的计算"""


def gain_k(v):
    """
    计算目标船船舶领域半径参数
    :param v: 目标船航速
    :return: 计算目标船船舶领域半径参数
    """
    k1 = 10 ** (0.3591 * math.log10(v) + 0.0952)
    k2 = 10 ** (0.5441 * math.log10(v) - 0.0795)
    return k1, k2


def quaternion(k1, k2, l):
    """
    计算目标船船舶领域的后方半径
    :param k1: 参数
    :param k2: 参数
    :param l: 目标船船长
    :return: 目标船船舶领域的后方半径
    """
    r_aft = (1 + 0.67 * math.sqrt(k1 ** 2 + (k2 / 2) ** 2)) * l * 3
    return r_aft


"""船舶向量夹角计算"""


def angle_of_vector(v1, v2):
    """
    计算向量间的夹角
    :param v1:向量
    :param v2: 向量
    :return:夹角
    """
    with np.errstate(invalid='ignore'):
        vector_prod = v1[0] * v2[0] + v1[1] * v2[1]
        length_prod = math.sqrt(pow(v1[0], 2) + pow(v1[1], 2)) * math.sqrt(pow(v2[0], 2) + pow(v2[1], 2))
        cos = vector_prod/length_prod
        angle = math.acos(cos)
        ang = angle * 180/np.pi
    return ang


def cr(v1, v2):  # 判断向量叉乘值的正负,以此来判断夹角正负
    """
    判断向量叉乘值
    :param v1: 向量
    :param v2: 向量
    :return: 叉乘值
    """
    z_theta = np.cross(v1, v2)
    return z_theta


def calculate_distance(x1, y1, x2, y2):
    """
    计算两点之间的距离
    :param x1: 1点的x坐标
    :param y1: 1点的y坐标
    :param x2: 2点的x坐标
    :param y2: 2点的y坐标
    :return: 两点距离(米)
    """
    # 计算水平距离和垂直距离
    horizontal_distance = abs(x2 - x1)
    vertical_distance = abs(y2 - y1)

    # 计算斜边距离（两点之间的直线距离）
    distance = math.sqrt(horizontal_distance ** 2 + vertical_distance ** 2)

    return distance


"""路径点的计算函数"""


def WaypointGet(cog_os, sog_os, x_os, y_os, cog_ts, sog_ts, x_ts, y_ts):
    """
    计算本船路径点
    :param cog_os: 本船航向
    :param sog_os: 本船航速
    :param x_os: 本船x坐标
    :param y_os: 本船x坐标
    :param cog_ts: 目标船航向
    :param sog_ts: 目标船航速
    :param x_ts: 目标船x坐标
    :param y_ts: 目标船y坐标
    :return: 本船的路径点,本船五海里内风险最大目标船的索引值,5海里范围内船舶的经纬度,5海里范围内船舶的dcpa,tcpa
    """
    L = 160  # 船长
    n_mile = 1852
    speed_second = 0.514444
    dcpa_lim = 0.5 * n_mile
    tcpa_lim = 600

    # -------------------------------------------------------------------------------------------------------
    # 识别5海里内的目标船

    x_recognizable = []  # 出现在本船一定范围内的可以识别的目标船的x坐标
    y_recognizable = []  # 出现在本船一定范围内的可以识别的目标船的y坐标
    cog_recognizable = []  # 出现在本船一定范围内的可以识别的目标船的航向
    sog_recognizable = []  # 出现在本船一定范围内的可以识别的目标船的航速
    dcpa_recognizable = []
    tcpa_recognizable = []
    for i in range(len(x_ts)):
        distance = calculate_distance(x_ts[i], y_ts[i], x_os, y_os)
        if distance < 5 * n_mile:
            x_recognizable.append(x_ts[i])
            y_recognizable.append(y_ts[i])
            cog_recognizable.append(cog_ts[i])
            sog_recognizable.append(sog_ts[i])
    # -------------------------------------------------------------------------------------------------------
    # -------------------------------------------------------------------------------------------------------
    # 判断会遇场景并且计算路径

    collision_avoidance_logic = []
    closest_point_of_approach = []
    angle_os_ts_pos = []  # 本船航向与目标船与本船相对位置的夹角
    angle_os_ts_cog = []  # 本船航向与目标船航向的夹角
    for i in range(len(x_recognizable)):
        # -------------------------------------------------------------------------------------------------------
        # -------------------------------------------------------------------------------------------------------
        # 计算本船与目标船的dcpa和tcpa（单位：m, s）
        ship_os = (x_os, y_os, sog_os * speed_second, cog_os)
        ship_ts = (x_recognizable[i], y_recognizable[i], sog_recognizable[i] * speed_second, cog_recognizable[i])
        dcpa, tcpa = calculate_DCPA_TCPA(ship_os, ship_ts)
        dcpa = round(dcpa, 2)
        tcpa = round(tcpa, 2)
        dcpa_recognizable.append(dcpa)
        tcpa_recognizable.append(tcpa)
        kad, kdt = gain_k(sog_ts[i])
        radius_after = quaternion(kad, kdt, L)
        # -------------------------------------------------------------------------------------------------------
        # 计算夹角值
        v_ts = np.array([sog_recognizable[i] * math.sin(math.radians(cog_recognizable[i])), sog_recognizable[i] * math.cos(math.radians(cog_recognizable[i])), 0])
        v_os = np.array([sog_os * math.sin(math.radians(cog_os)), sog_os * math.cos(math.radians(cog_os)), 0])
        pos_os_ts = np.array([x_recognizable[i] - x_os, y_recognizable[i] - y_os, 0])  # 本船与目标船相对位置向量
        angle_v_p = angle_of_vector(v_os, pos_os_ts)
        angle_v_ot = angle_of_vector(v_os, v_ts)
        z1 = cr(v_os, pos_os_ts)[2]
        z2 = cr(v_os, v_ts)[2]   # z是判断叉乘值的正负来判断角度的正负
        # -------------------------------------------------------------------------------------------------------
        # 判断夹角值正负（航向/位置，航向/航向：本船/目标船）

        if z1 > 0:
            angle_v_p = -angle_v_p
        elif z1 == 0:
            angle_v_p = 180
        else:
            angle_v_p = angle_v_p
        if z2 > 0:
            angle_v_ot = -angle_v_ot
        elif z2 == 0:
            angle_v_ot = 180
        else:
            angle_v_ot = angle_v_ot
        # -------------------------------------------------------------------------------------------------------
        # 根据夹角判断会遇场景

        #  1、追越场景
        if (-5 <= angle_v_ot <= 5) and (sog_os > sog_ts[i]) and (-90 < angle_v_p < 90):
            cpa = ([x_recognizable[i] + (sog_recognizable[i] * speed_second * math.sin(math.radians(cog_recognizable[i])) * tcpa),
                    y_recognizable[i] + (sog_recognizable[i] * speed_second * math.cos(math.radians(cog_recognizable[i])) * tcpa), 0])
            closest_point_of_approach.append(cpa)
            collision_avoidance_logic.append(3)
        #  2、对遇场景
        elif (-180 < angle_v_ot <= -175 or 175 <= angle_v_ot <= 180) and (-90 < angle_v_p < 90):
            cpa = ([x_recognizable[i] + (sog_recognizable[i] * speed_second * math.sin(math.radians(cog_recognizable[i])) * tcpa),
                    y_recognizable[i] + (sog_recognizable[i] * speed_second * math.cos(math.radians(cog_recognizable[i])) * tcpa), 0])
            closest_point_of_approach.append(cpa)
            collision_avoidance_logic.append(3)
        #  3、交叉会遇场景
        else:
            if (0 < dcpa < dcpa_lim) and 0 < tcpa < tcpa_lim:
                if 112.5 > angle_v_p > -5:
                    cal = 1  # 本船执行让路操作，从目标船船尾经过
                else:
                    cal = 0  # 本船执行直航操作，从目标船船头经过
            else:
                cal = 0
            # 计算会遇点位置cpa
            cpa = ([x_recognizable[i] + (sog_recognizable[i] * speed_second * math.sin(math.radians(cog_recognizable[i])) * tcpa),
                    y_recognizable[i] + (sog_recognizable[i] * speed_second * math.cos(math.radians(cog_recognizable[i])) * tcpa), 0])
            closest_point_of_approach.append(cpa)
            collision_avoidance_logic.append(cal)
        angle_os_ts_pos.append(angle_v_p)
        angle_os_ts_cog.append(angle_v_ot)
    # -------------------------------------------------------------------------------------------------------
    # -------------------------------------------------------------------------------------------------------
    # 根据最近会遇点计算路径点
    d_min = 2**20
    min_index = 'none'
    waypoint = [n_mile * math.sin(math.radians(cog_os)), n_mile * math.cos(math.radians(cog_os))]
    if len(x_recognizable) == 0:
        waypoint = [n_mile * math.sin(math.radians(cog_os)), n_mile * math.cos(math.radians(cog_os))]
        min_index = 'none'
    elif len(set(dcpa_recognizable)) == 1 and dcpa_recognizable[0] == float('inf'):
        waypoint = [n_mile * math.sin(math.radians(cog_os)), n_mile * math.cos(math.radians(cog_os))]
        min_index = 'none'
    else:
        for i in range(len(x_recognizable)):
            if dcpa_recognizable[i] == float('inf'):
                continue
            else:
                #  1、计算追越场景路径点
                if (-5 <= angle_os_ts_cog[i] <= 5) and (sog_os > sog_ts[i]) and (-90 < angle_os_ts_pos[i] < 90):
                    if dcpa_recognizable[i] == 0:
                        d_min = dcpa_recognizable[i]
                        min_index = i
                        waypoint = [(n_mile - 2 * dcpa_recognizable[min_index]) * math.sin(math.radians(cog_os + 30)),
                                    (n_mile - 2 * dcpa_recognizable[min_index]) * math.cos(math.radians(cog_os + 30))]
                    elif (0 < dcpa_recognizable[i] < dcpa_lim) and (dcpa_recognizable[i] < d_min) and 0 < \
                            tcpa_recognizable[i] < tcpa_lim:
                        d_min = dcpa_recognizable[i]
                        min_index = i
                        if angle_os_ts_pos[min_index] < 0:
                            waypoint = [(n_mile - 2 * dcpa_recognizable[min_index]) * math.sin(math.radians(cog_os + 30)),
                                        (n_mile - 2 * dcpa_recognizable[min_index]) * math.cos(math.radians(cog_os + 30))]
                        elif angle_os_ts_pos[min_index] > 0:
                            waypoint = [(n_mile + 2 * dcpa_recognizable[min_index]) * math.sin(math.radians(cog_os + 30)),
                                        (n_mile + 2 * dcpa_recognizable[min_index]) * math.cos(math.radians(cog_os + 30))]
                #  2、计算对遇场景路径点
                elif (-180 <= angle_os_ts_cog[i] <= -175 or 175 <= angle_os_ts_cog[i] <= 180) and (
                        -90 < angle_os_ts_pos[i] < 90):
                    if (0 <= dcpa_recognizable[i] < dcpa_lim) and (dcpa_recognizable[i] < d_min) and (
                            0 < tcpa_recognizable[i] < tcpa_lim):
                        d_min = dcpa_recognizable[i]
                        min_index = i
                        if angle_os_ts_pos[min_index] <= 0:
                            waypoint = [(n_mile - 2 * dcpa_recognizable[min_index]) * math.sin(math.radians(cog_os + 30)),
                                        (n_mile - 2 * dcpa_recognizable[min_index]) * math.cos(math.radians(cog_os + 30))]
                        elif angle_os_ts_pos[min_index] > 0:
                            waypoint = [(n_mile + 2 * dcpa_recognizable[min_index]) * math.sin(math.radians(cog_os + 30)),
                                        (n_mile + 2 * dcpa_recognizable[min_index]) * math.cos(math.radians(cog_os + 30))]
                #  3、计算交叉场景路径点
                elif (collision_avoidance_logic[i] == 1) and (-180 < angle_os_ts_cog[i] < 0) and (
                        0 <= angle_os_ts_pos[i] <= 112.5) and (0 < tcpa_recognizable[i] < tcpa_lim):
                    if dcpa_recognizable[i] < dcpa_lim:
                        d_min = dcpa_recognizable[i]
                        min_index = i
                        waypoint = [closest_point_of_approach[min_index][0] - radius_after * math.sin(
                            math.radians(cog_ts[min_index])),
                                    closest_point_of_approach[min_index][1] - radius_after * math.cos(
                                        math.radians(cog_ts[min_index]))]
                        if 67.5 < angle_os_ts_pos[min_index] < 112.5:
                            waypoint = [n_mile * math.sin(math.radians(cog_os)), n_mile * math.cos(math.radians(cog_os))]
                elif (collision_avoidance_logic[i] == 1) and (0 < angle_os_ts_cog[i] < 180) and (
                        -5 <= angle_os_ts_pos[i] <= 0) and (0 < tcpa_recognizable[i] < tcpa_lim):
                    if dcpa_recognizable[i] < dcpa_lim:
                        d_min = dcpa_recognizable[i]
                        min_index = i
                        waypoint = [closest_point_of_approach[min_index][0] - radius_after * math.sin(
                            math.radians(cog_ts[min_index])),
                                    closest_point_of_approach[min_index][1] - radius_after * math.cos(
                                        math.radians(cog_ts[min_index]))]
                #  4、无风险路径点
                else:
                    waypoint = [n_mile * math.sin(math.radians(cog_os)), n_mile * math.cos(math.radians(cog_os))]
                    min_index = 'none'

    return waypoint, min_index, x_recognizable, y_recognizable, dcpa_recognizable, tcpa_recognizable, collision_avoidance_logic, closest_point_of_approach


# if __name__ == "__main__":
#     me_listening_socket = None
#     remote_ip = ('127.0.0.1')
#     me_listening_port = 8090  # 本脚本（控制器）监听的端口号
#     remote_port = int(8080)  # 远端程序（模拟器）监听的端口号
#     # me_listening_port = 60001  # 本脚本（控制器）监听的端口号
#     # remote_port = int(60000)  # 远端程序（模拟器）监听的端口号
#     me_sending_port = 50000  # 本demo发出数据使用的端口
#
#
#     def listen_channel_func():
#         # ================================
#         # 与目标端口建立通信
#         # ================================
#         try:
#             print("建立监听,本地监听端口号是：%s" % (me_listening_port,))
#             me_listening_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#             me_listening_socket.bind(('', me_listening_port))
#             # ================================================================================================
#             # ================================================================================================
#             # 首次接收数据
#             receive_data, remote_address = me_listening_socket.recvfrom(1024)
#             a = receive_data.decode('utf-8')
#
#             # 数据处理
#             (norx, hdsRudderPS, hdsRudderSB, hdsTelePS, hdsTeleSB, hdsThrBow, hdsThrStern, Latitude, Longitude,
#              LateralSpeed, LongitudinalSpeed, Heading, NEDe, NEDn, Pitch, Roll, nory) = a.split(',')
#             lat_os_start = float(Latitude)
#             lon_os_start = float(Longitude)
#             ship_generator = TargetShipSet.target_ship(lat_os_start, lon_os_start)
#             # 获得本船首个位置
#             source_epsg = 'EPSG:4326'  # WGS84经纬度坐标系
#             target_epsg = 'EPSG:3857'  # 地理坐标参考系统
#             while True:
#                 # ================================================================================================
#                 # ================================================================================================
#                 # 接收模拟器本船的数据
#                 # 获得自定义目标船数据并完成坐标系转换
#                 receive_data, remote_address = me_listening_socket.recvfrom(1024)
#                 a = receive_data.decode('utf-8')
#
#                 # 数据处理
#                 # print('使用逗号获取子串:', a.split(','))
#                 (norx, hdsRudderPS, hdsRudderSB, hdsTelePS, hdsTeleSB, hdsThrBow, hdsThrStern, Latitude, Longitude,
#                  LateralSpeed, LongitudinalSpeed, Heading, NEDe, NEDn, Pitch, Roll, nory) = a.split(',')
#                 lat_os = float(Latitude)
#                 lon_os = float(Longitude)
#                 cog_os = float(Heading)
#                 lat_speed = float(LateralSpeed)
#                 lon_speed = float(Longitude)
#                 roll_os = float(Roll)
#                 sog_os = np.sqrt(lat_speed ** 2 + lon_speed ** 2)
#                 NEDe = float(NEDe)
#                 NEDn = float(NEDn)
#                 x_os, y_os = TargetShipSet.convert_latlon_to_xy(lat_os, lon_os, source_epsg, target_epsg)
#                 x_ts, y_ts, cog_ts, sog_ts = next(ship_generator)
#                 for i in range(len(x_ts)):
#                     x_ts[i] = x_ts[i] - x_os
#                     y_ts[i] = y_ts[i] - y_os
#                 x_os = x_os - x_os
#                 y_os = y_os - y_os
#                 waypoint, min_index, x_recognizable, y_recognizable, dcpa_recognizable, tcpa_recognizable = WaypointGet(
#                     cog_os, sog_os, x_os, y_os,
#                     cog_ts, sog_ts, x_ts, y_ts)
#                 print(dcpa_recognizable, tcpa_recognizable)
#         except:
#             print("建立监听失败，退出监听remote数据")
#         finally:
#             print("建立监听成功！")
#             if me_listening_socket is not None:
#                 me_listening_socket.close()
#                 me_listening_socket = None
#     listen_channel_func()

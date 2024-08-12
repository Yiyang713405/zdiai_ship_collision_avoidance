# -------------------------------------------------------------------------------------
# import 此demo需要使用的包
# --------------------
import socket
import pyproj
from pynput import keyboard
import csv
import time
import numpy as np
import matplotlib.pyplot as plt
from Tools import TargetShipSet
from Tools import WaypointGet
from Tools import shipmodle
from Tools import EvaluationFunction
from Tools import PointDelete
from service import ip_path_planning
from service import ip_heading_planning
from service import ip_controller
from service import ip_MFAC
from service import ip_guidance
from service.tanker_VLCC8_L333 import VLCC8L333

from PlanningAlgorithm import PotentialFieldPlanning
from PlanningAlgorithm import bezier_path

import math

# --------------------
# import 控制器和规划器
# --------------------
# import dzl_heading_planning
# import ip_class
# import ip_controller

# -------------------------------------------------------------------------------------
# 初始化变量
# --------------------
# 初始化通信地址和端口
me_listening_socket = None
remote_ip = ('127.0.0.1')
me_listening_port = 8090  # 本脚本（控制器）监听的端口号
remote_port = int(8080)  # 远端程序（模拟器）监听的端口号
# me_listening_port = 60001  # 本脚本（控制器）监听的端口号
# remote_port = int(60000)  # 远端程序（模拟器）监听的端口号
me_sending_port = 50000  # 本demo发出数据使用的端口

# From YuYue
# me_listening_port = 60000  # 本脚本（控制器）监听的端口号
# me_listening_socket = None
# # --------------------
# remote_ip = ('127.0.0.1')
# remote_port = int(60001)  # 远端程序（模拟器）监听的端口号
# me_sending_port = 50000  # 本demo发出数据使用的端口
# -------------------------------------------------------------------------------------

# -------------------------------------------------------------------------------------
# 定义模拟器反馈状态-全局变量（有多少个数据出来就要定义 x + 2 个）（以字符串格式）
# --------------------
norx = ''  # 帧头
nory = ''  # 帧尾
hdsRudderPS = ''
hdsRudderSB = ''
hdsTelePS = ''
hdsTeleSB = ''
hdsThrBow = ''
hdsThrStern = ''

# --------------------
# 单独定义本测试对象所需要数据变量
# --------------------
Latitude = ''
Longitude = ''
LateralSpeed = ''
LongitudinalSpeed = ''
# R_Yawrate = ''
Heading = ''
NEDe = ''
NEDn = ''
Pitch = ''
Roll = ''

t = time.ctime()
# --------------------
# 单独定义本测试其余变量
# --------------------

# 坐标列表写出及开启画图窗口
l = 27.4
L = 16.4
w = 11.5
List_e = []  # 定义一个 东向位移 的空列表用来接收动态的数据
List_n = []  # 定义一个 北向位移 的空列表用来接收动态的数据
plt.ion()  # 开启一个hold on的窗口
r1 = 0
r2 = 0
t1 = 0
t2 = 0

ship_param = VLCC8L333()


# def on_press(key):
#     '按下按键时执行。'
    # try:
    #     print('alphanumeric key {0} pressed'.format(key.char))
    #
    # except AttributeError:
    #     print('special key {0} pressed'.format(key))
    # 通过属性判断按键类型。


def on_release(key):
    '松开按键时执行。'
    global r1, r2, t1, t2
    # global r1,t1
    # print('{0} released'.format(key))
    # FIXME 单桨单舵船舶，控制q和a是加减舵角，i和k是加减转速
    # FIXME 舵角范围是正负35度，转速是正负10
    if format(key.char) == 'q':
        r1 += 1
        print('Port Rudder', r1)
    elif format(key.char) == 'w':
        r2 += 1
        print('StarB Rudder', r2)
    elif format(key.char) == 'a':
        r1 -= 1
        print('Port Rudder', r1)
    elif format(key.char) == 's':
        r2 -= 1
        print('StarB Rudder', r2)
    elif format(key.char) == 'u':
        t1 += 1
        print('Port Tele', t1)
    elif format(key.char) == 'i':
        t2 += 1
        print('StarB Tele', t2)
    elif format(key.char) == 'j':
        t1 -= 1
        print('Port Tele', t1)
    elif format(key.char) == 'k':
        t2 -= 1
        print('StarB Tele', t2)
    else:
        print('nor')
    if key == keyboard.Key.esc:
        # Stop listener
        return False


def listen_channel_func():
    # ================================
    # 与目标端口建立通信
    # ================================
    try:
        print("建立监听,本地监听端口号是：%s" % (me_listening_port,))
        me_listening_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        me_listening_socket.bind(('', me_listening_port))
        # ================================================================================================
        # ================================================================================================
        # 首次接收数据
        receive_data, remote_address = me_listening_socket.recvfrom(1024)
        a = receive_data.decode('utf-8')

        # 数据处理
        (norx, hdsRudderPS, hdsRudderSB, hdsTelePS, hdsTeleSB, hdsThrBow, hdsThrStern, Latitude, Longitude,
         LateralSpeed, LongitudinalSpeed, Heading, NEDe, NEDn, Pitch, Roll, nory) = a.split(',')
        lat_os_start = float(Latitude)
        lon_os_start = float(Longitude)
        ship_generator = TargetShipSet.target_ship(lat_os_start, lon_os_start)
        # 获得本船首个位置
        # ================================================================================================
        # ================================================================================================
        # 开始循环接受处理数据
        plt.ion()
        fig, axes = plt.subplots()
        plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [exit(0) if event.key == 'escape' else None])
        start_point = [0, 0]
        start_x = start_point[0]
        start_y = start_point[1]
        Mile2m = 1852
        source_epsg = 'EPSG:4326'  # WGS84经纬度坐标系
        target_epsg = 'EPSG:3857'  # 地理坐标参考系统
        while True:
            # ================================================================================================
            # ================================================================================================
            # 接收模拟器本船的数据
            # 获得自定义目标船数据并完成坐标系转换
            receive_data, remote_address = me_listening_socket.recvfrom(1024)
            a = receive_data.decode('utf-8')

            # 数据处理
            # print('使用逗号获取子串:', a.split(','))
            (norx, hdsRudderPS, hdsRudderSB, hdsTelePS, hdsTeleSB, hdsThrBow, hdsThrStern, Latitude, Longitude,
             LateralSpeed, LongitudinalSpeed, Heading, NEDe, NEDn, Pitch, Roll, nory) = a.split(',')
            lat_os = float(Latitude)
            lon_os = float(Longitude)
            cog_os = float(Heading)
            lat_speed = float(LateralSpeed)
            lon_speed = float(Longitude)
            roll_os = float(Roll)
            sog_os = np.sqrt(lat_speed ** 2 + lon_speed ** 2)
            print("cog_os,sog_os:", cog_os, sog_os)
            NEDe = float(NEDe)
            NEDn = float(NEDn)
            x_os, y_os = TargetShipSet.convert_latlon_to_xy(lat_os, lon_os, source_epsg, target_epsg)
            x_ts, y_ts, cog_ts, sog_ts = next(ship_generator)
            for i in range(len(x_ts)):
                x_ts[i] = x_ts[i] - x_os
                y_ts[i] = y_ts[i] - y_os
            x_os = x_os - x_os
            y_os = y_os - y_os
            # ================================================================================================
            # ================================================================================================
            # 计算路径点
            (waypoint, min_index, x_recognizable, y_recognizable, dcpa_recognizable, tcpa_recognizable,
             cal, cpa) = WaypointGet.WaypointGet(cog_os, sog_os, x_os, y_os, cog_ts, sog_ts, x_ts, y_ts)
            # print(closest_point_of_approach)
            icon_all, icon_risk, icon_cpa = shipmodle.shipmodle(cog_ts, x_ts, y_ts, cog_os, x_os, y_os)
            # ================================================================================================
            # ================================================================================================
            # 路径规划算法接入

            obstacles_y = y_ts
            obstacles_x = x_ts
            end_point = waypoint
            goal_x = end_point[0]
            goal_y = end_point[1]

            '''---------------------APF---start------------------------------'''
            # grid_size = 0.1  # Resolution of the potential rho (m)
            # area_radius = 0.5  # Potential area radius (m)
            # rx_apf, ry_apf = PotentialFieldPlanning.potential_field_planning(start_x, start_y, goal_x, goal_y,
            #                                                                  obstacles_x,
            #                                                                  obstacles_y, grid_size, area_radius)
            '''---------------------APF---end---------------------------------'''
            '''---------------------AStar---start-----------------------------'''
            # ASPlanner = a_star.AStarPlanner(obstacles_x, obstacles_y, grid_size, area_radius)
            # rx_astar, ry_astar = ASPlanner.planning(start_x, start_y, goal_x, goal_y)
            '''---------------------AStar---end-----------------------------'''
            '''---------------------bezier_path---start------------------------------'''
            print(cog_os)
            angle_trans = (360 - cog_os) % 360
            angle_trans = (angle_trans + 90) % 360  # 将顺时针纵轴向上为0°的角度数据转化为逆时针横轴向右为0°的角度数据
            start_yaw = np.radians(angle_trans)
            end_yaw = start_yaw
            path, control_points = bezier_path.calc_4points_bezier_path(start_x, start_y, start_yaw, goal_x, goal_y, end_yaw, offset=3)
            rx_bezier = path.T[0]
            ry_bezier = path.T[1]
            '''---------------------bezier_path---start------------------------------'''
            rx = rx_bezier
            ry = ry_bezier
            rxy = np.vstack((rx, ry)).T
            # lenth = len(rx)
            # rx_for_redis = rx_apf * Mile2m
            # ry_for_redis = ry_apf * Mile2m
            path_length = EvaluationFunction.cal_path_length(rx, ry)
            print('path length=', path_length)
            # select_num = 15
            select_num = round(path_length / ship_param.hull.LPP)
            print('select num=', select_num)
            rxfinal = EvaluationFunction.select_elements(rx, select_num)
            ryfinal = EvaluationFunction.select_elements(ry, select_num)
            rxyfinal = np.vstack((rxfinal, ryfinal)).T

            # ================================================================================================
            # ================================================================================================
            # 控制算法接入
            ship_param.e = x_os
            ship_param.n = y_os
            ship_param.psi_deg = cog_os

            OS_state = [0, 0, cog_os, sog_os]
            track_point = rxy[1, :]
            h = 10
            psi_controller = ip_MFAC.RO_MFAC(rho=1.8, lamda=0.26, eta=0.36, mu=0.5, phi1=0.1, epsilon=0.001, K1=13,
                                             output_min=-math.pi / 12, output_max=math.pi / 12)
            heading_cmd, speed_cmd = ip_guidance.Point_track().solve(OS_state, track_point, h)
            print(heading_cmd, speed_cmd)
            delta_cmd_deg = psi_controller.solve(np.deg2rad(heading_cmd), np.deg2rad(cog_os), np.deg2rad(roll_os))
            ship_rpm = 10

            # path_plan = ip_path_planning.Path()
            # path_plan.add_point(rxyfinal)
            # heading_planner = ip_heading_planning.LOS()
            # heading_controller = ip_controller.PID(sample_time=0.01, kp=3.0, ki=0.03, kd=0.06,
            #                                        output_min=-35.0, output_max=35.0)
            # ship_param.psi_cmd_deg = heading_planner.solve(ship_param, path_plan)
            # print(ship_param.psi_cmd_deg)
            # ship_param.rudder.delta_cmd_deg = heading_controller.solve(ship_param.psi_cmd_deg, ship_param.psi_deg)
            # print(ship_param.rudder.delta_cmd_deg)
            # ship_rpm = 10

            # # 基础显示：
            # print("接收到的原始数据为:    %s" % a)
            # print("接收到的原始数据为:    %s" % data)
            # print('当前航向：  %s     当前东向位移与北向位移： %s    %s' % (Heading, NEDe, NEDn))  #输出变量
            # print('LOS计算航向角：   %f' % psi_cmd_deg)
            # print('PID计算控制舵角:   %f' % delta_cmd_deg)
            # # delta_cmd_deg = 7
            #
            # # ================================
            # #发送算法指令给目标端口
            send_channel_func(delta_cmd_deg, ship_rpm)

            # ================================================================================================
            # ================================================================================================
            # 绘图
            x_os = x_os / Mile2m
            y_os = y_os / Mile2m
            x_ts = [element / Mile2m for element in x_ts]
            y_ts = [element / Mile2m for element in y_ts]
            if len(x_recognizable) != 0:
                x_recognizable = [element / Mile2m for element in x_recognizable]
                y_recognizable = [element / Mile2m for element in y_recognizable]
            rx = [element / Mile2m for element in rx]
            ry = [element / Mile2m for element in ry]
            waypoint_plot = [element / Mile2m for element in waypoint]
            # print(waypoint_plot, min_index)

            axes.clear()
            axes.axis('equal')
            for i in range(len(x_ts)):
                axes.scatter(x_ts[i], y_ts[i], marker=icon_all[i], s=10, facecolor="none", edgecolors="black")
                axes.text(x_ts[i], y_ts[i] + 0.2, f"Target{i + 1}", ha="center", family='sans-serif', size=8)
            axes.scatter(x_os, y_os, marker=icon_all[-1], s=10, facecolor="none", edgecolors="black")
            if len(x_recognizable) != 0:
                for i in range(len(x_recognizable)):
                    axes.scatter(x_recognizable[i], y_recognizable[i], marker=icon_risk[i], s=10, facecolor="orange",
                                 edgecolors="black")
                    axes.text(x_recognizable[i]+0.3, y_recognizable[i]-0.3,
                              f"dcpa:{dcpa_recognizable[i]} m\n tcpa:{tcpa_recognizable[i]} s\n cal:{cal[i]}",
                              ha="center", family='sans-serif', size=8)
                    if min_index != 'none':
                        axes.scatter(x_recognizable[min_index], y_recognizable[min_index], marker=icon_risk[min_index],
                                     s=10, facecolor="red", edgecolors="black")
                        if cal[i] == 1:
                            axes.scatter(cpa[min_index][0] / Mile2m, cpa[min_index][1] / Mile2m,
                                         marker=icon_cpa[min_index],
                                         s=10, facecolor="none", edgecolors="black", linestyle='--')
            axes.scatter(x_os, y_os, s=10, alpha=0.3, marker=icon_all[-1], facecolor="black", edgecolors="black")
            axes.text(x_os, y_os - 0.2, 'OwnShip', ha="center", family='sans-serif', size=8)
            axes.scatter(waypoint_plot[0], waypoint_plot[1], s=50, marker='*', color='r')
            axes.text(waypoint_plot[0], waypoint_plot[1] + 0.1, f'WayPoint_{min_index}', ha="center", family='sans-serif', size=8)
            deg = list(range(0, 360, 5))
            deg.append(0)
            xl2 = [1000 / Mile2m * math.cos(np.deg2rad(d)) for d in deg]
            yl2 = [1000 / Mile2m * math.sin(np.deg2rad(d)) for d in deg]
            plt.plot(xl2, yl2, 'r')
            axes.plot(rx, ry, color="b", linestyle="-", label="Planned Path")
            axes.grid(True)
            axes.grid(color='black', linestyle='--', linewidth=0.3, alpha=0.3)
            axes.set_xlabel('x(n mile)')
            axes.set_ylabel('y(n mile)')
            axes.legend(loc="upper left")
            axes.grid(True)
            # 显示图形并暂停1秒
            waypoint.clear()
            x_ts.clear()
            y_ts.clear()
            plt.pause(1)
            axes.clear()  # 清除之前的图形
            # # ================================
    except:
        print("建立监听失败，退出监听remote数据")
    finally:
        print("建立监听成功！")
        if me_listening_socket is not None:
            me_listening_socket.close()
            me_listening_socket = None


def send_channel_func(delta_cmd_deg, rpm):
    # ================================
    # sending part for send data
    # ================================
    global remote_ip
    global remote_port
    global me_sending_port

    me_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    me_socket.bind(('', me_sending_port))

    # ================================
    # 代入端口接受格式
    # ================================
    msg = ('$LUPWM,' + str(delta_cmd_deg) + ',' + str(r2) + ',' + str(t1) + ',' + str(rpm) + '\x0a')  # 控拖轮
    # msg = ('$LUPWM,' + str(r1) + ',' + str(t1) + '\x0a')
    print(msg)
    me_socket.sendto(msg.encode("utf-8"), (remote_ip, remote_port))

    if me_socket is not None:
        me_socket.close()
        me_socket = None


if __name__ == "__main__":
    # # # X = listen_channel_func()
    # listen_thread = Thread(target=listen_channel_func)
    # listen_thread.start()
    #
    # with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    #     listener.join()
    listen_channel_func()

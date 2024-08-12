# -------------------------------------------------------------------------------------
# import 此demo需要使用的包
# --------------------
import socket
import time
from threading import Thread
import csv
import time
import numpy as np
import matplotlib.pyplot as plt
# --------------------
# import 控制器和规划器
# --------------------
# import dzl_heading_planning
# import ip_class
# import ip_controller
import sys

# -------------------------------------------------------------------------------------
# 初始化变量
# --------------------
# 初始化通信地址和端口
me_listening_port = 60000  # 本脚本（控制器）监听的端口号
me_listening_socket = None
# --------------------
remote_ip = ('127.0.0.1')
remote_port = int(60001)  # 远端程序（模拟器）监听的端口号
me_sending_port = 50000  # 本demo发出数据使用的端口
# -------------------------------------------------------------------------------------

# -------------------------------------------------------------------------------------
# 定义模拟器反馈状态-全局变量（有多少个数据出来就要定义 x + 2 个）（以字符串格式）
# --------------------
norx = ''  # 帧头
nory = ''  # 帧尾
nor1 = ''
nor2 = ''
nor3 = ''
nor4 = ''
nor5 = ''
nor6 = ''
nor7 = ''
nor8 = ''
nor9 = ''
nor10 = ''
# --------------------
# 单独定义本测试对象所需要数据变量
# --------------------
R_Yawrate = ''
R_Heading = ''
R_NEDe = ''
R_NEDn = ''
t = time.ctime()
# --------------------
# 单独定义本测试其余变量
# --------------------
Vessel_LPP = 27.4
test_points = [[50, 50], [100, 90], [150, 130], [200, 160], [250, 190], [300, 210], [350, 200]]
# test_points = [[160, 200], [240, 240], [400, 400], [600, 400], [700, 572], [600, 746], [400, 746], [300, 572], [400, 400]]
# path_cmd = ip_class.Path()
# path_cmd.add_point(test_points)

# 坐标列表写出及开启画图窗口
List_e = []  # 定义一个 东向位移 的空列表用来接收动态的数据
List_n = []  # 定义一个 北向位移 的空列表用来接收动态的数据
plt.ion()  # 开启一个hold on的窗口


# plt.plot(path_cmd.coord[:, 1], path_cmd.coord[:, 0], color="r", linestyle="", marker="o", markersize=5, label="REF")

def listen_channel_func():
    global norx
    global nory
    global nor1
    global nor2
    global nor3
    global nor4
    global nor5
    global nor6
    global nor7
    global nor8
    global nor9
    global nor10
    global R_Heading
    global R_NEDe
    global R_NEDn
    global t
    global me_listening_socket

    # ================================
    # 与目标端口建立通信
    # ================================
    try:
        print("建立监听,本地监听端口号是：%s" % (me_listening_port,))
        me_listening_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        me_listening_socket.bind(('', me_listening_port))

        # 将字符串数据写入csv文件
        with open('test_new.csv', 'a', newline='') as file:
            writer1 = csv.writer(file)
            writer1.writerow(['\n'])
            writerow = writer1.writerow(["Start", t])
        file = open('test_new.csv', 'a', newline='')
        writer = csv.writer(file)
        # 参数定义
        prev_R_Yawrate = 0
        stability_counter = 0
        stability_threshold = 0.0002   # 波动范围
        step = 40
        iteration_count = 0
        delta_cmd_deg_left = -90
        delta_cmd_deg_right = 90
        # 开始循环接受处理数据
        while True:
            # 接收数据
            receive_data, remote_address = me_listening_socket.recvfrom(1024)
            a = receive_data.decode('utf-8')
            # 数据处理
            # print('使用逗号获取子串:', a.split(','))
            norx, OPangle, OSangle, OPthrust, OSthrust, R_Lat, R_Lon, R_LatSpeed, R_LonSpeed, R_Yawrate, R_Heading, R_NEDe, R_NEDn, FPrudder, FSrudder, FPRPM, FSRPM, nory = a.split(
                ',')
            writer.writerow(a.split(','))
            # 开始循环
            # 计算 R_Yawrate 数据波动
            diff = float(R_Yawrate) - prev_R_Yawrate
            result = "{:.4f}".format(diff)
            if abs(diff) <= stability_threshold:
                stability_counter += 1
            else:
                stability_counter = 0
                prev_R_Yawrate = float(R_Yawrate)
            # 判断是否触发条件满足：
            if stability_counter >= step or iteration_count >= 1200:
                # 右舵递增到90
                # delta_cmd_deg_right += 5
                # stability_counter = 0
                # iteration_count = 0
                # if delta_cmd_deg_right >= 95:
                #     sys.exit()
                # 右舵递减到0
                delta_cmd_deg_right -= 5
                stability_counter = 0
                iteration_count = 0
                if delta_cmd_deg_right < 0:
                    sys.exit()
            iteration_count += 1
            # 发送数据，调用 send_channel_func 函数
            sent_msg, sent_me_socket = send_channel_func(delta_cmd_deg_left, delta_cmd_deg_right)
            # 打印发送的数据
            print('------------------------------------------------------------------------')
            print(f"Sent: {sent_msg}")
            print(f"当前船舶角速度：{float(R_Yawrate)}，波动范围：{result}， 当前波动已保持：{stability_counter}， 当前右舵下已运行：{iteration_count}")
            # 基础显示：
            print("接收到的原始数据为:    %s" % a)
            print('当前航向：  %s     当前东向位移与北向位移： %s    %s' % (R_Heading, R_NEDe, R_NEDn))  # 输出变量
            print('------------------------------------------------------------------------')
            # ================================
            # 发送算法指令给目标端口
            # send_channel_func(delta_cmd_deg)

            if receive_data is None or len(receive_data) == 0:
                # 没有收到数据的话，sleep 1秒
                time.sleep(1)
                continue

    except:
        print("建立监听失败，退出监听remote数据")
    finally:
        print("建立监听成功！")
        file.close('test_new.csv')
        if me_listening_socket is not None:
            me_listening_socket.close()
            me_listening_socket = None


def send_channel_func(delta_deg_left, delta_deg_right):
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
    msg = ('$LUPWM,' + str(delta_deg_left) + ',' + str(delta_deg_right) + ',' + '10' + ',' + '10' + '\x0a')
    me_socket.sendto(msg.encode("utf-8"), (remote_ip, remote_port))

    if me_socket is not None:
        me_socket.close()
        me_socket = None

    return msg, me_socket


if __name__ == "__main__":
    x = listen_channel_func()

# -------------------------------------------------------------------------------------
# import 此demo需要使用的包
# --------------------
import socket
import time
from threading import Thread
from pynput import keyboard
import csv
import time
import numpy as np
import matplotlib.pyplot as plt
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
# -------------------------------------------------------------------------------------

# -------------------------------------------------------------------------------------
# 定义模拟器反馈状态-全局变量（有多少个数据出来就要定义 x + 2 个）（以字符串格式）
# --------------------
norx = ''      #帧头
nory = ''      #帧尾
hdsRudderPS= ''
hdsRudderSB= ''
hdsTelePS= ''
hdsTeleSB= ''
hdsThrBow= ''
hdsThrStern= ''


# --------------------
# 单独定义本测试对象所需要数据变量
# --------------------
Latitude = ''
Longitude = ''
LateralSpeed = ''
LongitudinalSpeed = ''
# R_Yawrate = ''
Heading= ''
NEDe= ''
NEDn= ''
Pitch= ''
Roll= ''

t = time.ctime()
# --------------------
# 单独定义本测试其余变量
# --------------------

#坐标列表写出及开启画图窗口
l = 27.4
L = 16.4
w = 11.5
List_e = []                    # 定义一个 东向位移 的空列表用来接收动态的数据
List_n = []                    # 定义一个 北向位移 的空列表用来接收动态的数据
plt.ion()                      # 开启一个hold on的窗口
r1=0
r2=0
t1=0
t2=0

def on_press(key):
    '按下按键时执行。'

    # try:
    #     print('alphanumeric key {0} pressed'.format(key.char))
    #
    # except AttributeError:
    #     print('special key {0} pressed'.format(key))
    #通过属性判断按键类型。

def on_release(key):
    '松开按键时执行。'
    global r1,r2,t1,t2
    # global r1,t1
    # print('{0} released'.format(key))
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

        # 将字符串数据写入csv文件
        with open('ship_date1.csv', 'a', newline='') as file:
            writer1 = csv.writer(file)
            writer1.writerow(['\n'])
            writer1.writerow(["Start", t])
            writer1.writerow(['norx ', 'hdsRudderPS', 'hdsRudderSB', 'hdsTelePS', 'hdsTeleSB', 'hdsThrBow', 'hdsThrStern',
                              'Latitude', 'Longitude', 'LateralSpeed', 'LongitudinalSpeed', 'Heading', 'NEDe', 'NEDn', 'Pitch', 'Roll', 'nory'])
            # writer1.writerow(['norx ', 'Latitude', 'Longitude', 'Heading', 'Pitch', 'Roll', 'nory'])
        # 开始循环接受处理数据
        while True:
            data = []
            # 接收数据
            receive_data, remote_address = me_listening_socket.recvfrom(1024)
            a=receive_data.decode('utf-8')

            #数据处理
            # print('使用逗号获取子串:', a.split(','))
            norx , hdsRudderPS, hdsRudderSB, hdsTelePS, hdsTeleSB, hdsThrBow, hdsThrStern, Latitude, Longitude, LateralSpeed, LongitudinalSpeed, Heading, NEDe, NEDn, Pitch, Roll, nory = a.split(',')
            Latitude1 = float(Latitude) - 27.668179857849
            Longitude1 = float(Longitude) + 126.984403693418
            Latitude = str(Latitude1)
            Longitude = str(Longitude1)
            data.append(norx)
            data.append(Latitude)
            data.append(Longitude)
            data.append(Heading)
            data.append(Pitch)
            data.append(Roll)
            data.append(nory)

            with open('ship_date1.csv', 'a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(a.split(','))
                # writer.writerow(data)
            Heading = float(Heading)
            NEDe = float(NEDe)
            NEDn = float(NEDn)

            # # 控制算法嵌入
            # psi_cmd_deg = dzl_heading_planning.LOS().solve(Vessel_LPP, Heading, NEDe, NEDn, path_cmd)
            # delta_cmd_deg = ip_controller.PID(sample_time=0.01, kp=3.0, ki=0.03, kd=0.06, output_min=-35.0,
            #                                   output_max=35.0).solve(psi_cmd_deg, Heading)
            #
            # # 基础显示：
            print("接收到的原始数据为:    %s" % a)
            # print("接收到的原始数据为:    %s" % data)
            # print('当前航向：  %s     当前东向位移与北向位移： %s    %s' % (Heading, NEDe, NEDn))  #输出变量
            # print('LOS计算航向角：   %f' % psi_cmd_deg)
            # print('PID计算控制舵角:   %f' % delta_cmd_deg)
            # # delta_cmd_deg = 7
            #
            # # ================================
            # #发送算法指令给目标端口
            send_channel_func()
            # # ================================

            # # ================================
            # #画图
            #
            # plt.clf()
            # plt.plot([207, 233, 233, 207], [275, 275, 325, 325], 'o')  # （2，3），（3，4），（4，5），（5，6） 4个离散的点。
            # # 画连线
            # plt.plot([207, 233, 233, 207, 207], [275, 275, 325, 325, 275], linewidth=1,color='red')
            # plt.plot([215, 225, 225, 215, 215], [290, 290, 310, 310, 290], linewidth=1, color='blue')
            # plt.plot([0, 225.1, 225.1, 214.9, 0], [0, 0, 310.1, 310.1, 0], linewidth=1, color='green')
            # car_x = [NEDe + (l-11) * np.sin(Heading * math.pi / 180) / 2 - w * np.cos(Heading * math.pi / 180) / 2,
            #          NEDe + l * np.sin(Heading * math.pi / 180) / 2,
            #          NEDe + (l-11) * np.sin(Heading * math.pi / 180) / 2 + w * np.cos(Heading * math.pi / 180) / 2,
            #          NEDe - (l-11) * np.sin(Heading * math.pi / 180) / 2 + w * np.cos(Heading * math.pi / 180) / 2,
            #          NEDe - (l-11) * np.sin(Heading * math.pi / 180) / 2 - w * np.cos(Heading * math.pi / 180) / 2,
            #          NEDe + (l-11) * np.sin(Heading * math.pi / 180) / 2 - w * np.cos(Heading * math.pi / 180) / 2]
            # car_y = [NEDn + (l-11) * np.cos(Heading * math.pi / 180) / 2 + w * np.sin(Heading * math.pi / 180) / 2,
            #          NEDn + l * np.cos(Heading * math.pi / 180) / 2,
            #          NEDn + (l-11) * np.cos(Heading * math.pi / 180) / 2 - w * np.sin(Heading * math.pi / 180) / 2,
            #          NEDn - (l-11) * np.cos(Heading * math.pi / 180) / 2 - w * np.sin(Heading * math.pi / 180) / 2,
            #          NEDn - (l-11) * np.cos(Heading * math.pi / 180) / 2 + w * np.sin(Heading * math.pi / 180) / 2,
            #          NEDn + (l-11) * np.cos(Heading * math.pi / 180) / 2 + w * np.sin(Heading * math.pi / 180) / 2]
            # plt.plot(car_x, car_y, linewidth=1, color='red')
            # text_pt1 = plt.text(-31, 380, '', fontsize=11)
            # text_pt2 = plt.text(-31, 340, '', fontsize=11)
            # text_pt3 = plt.text(-31, 300, '', fontsize=11)
            # text_pt1.set_text('PR:{0} SR:{1}'.format(r1, t1))
            # text_pt2.set_text('U:{0}    V:{1}    heading:{3}    R:{2}'.format(LongitudinalSpeed[0:5], LateralSpeed[0:5], Heading[0:5], Heading[0:5]))
            # text_pt3.set_text('E_p:{0}      N_p:{1}'.format(NEDe[0:5], NEDn[0:5]))
            # List_e.append(NEDe)
            # List_n.append(NEDn)
            # plt.xlabel('E-Position/m')  # 设置 x 轴名字
            # plt.ylabel('N-Position/m')
            # plt.xlim(-50, 300)
            # plt.ylim(-50, 400)
            # plt.plot(List_e, List_n)  # 画出当前 ax 列表和 ay 列表中的值的图形
            # # plt.show
            # plt.pause(0.1)  # 暂停一秒
            # plt.ioff()  # 关闭画图的窗口


            if receive_data is None or len(receive_data) == 0:
                # 没有收到数据的话，sleep 1秒
                time.sleep(1)
                continue

    except:
        print("建立监听失败，退出监听remote数据")
    finally:
        print("建立监听成功！")
        if me_listening_socket is not None:
            me_listening_socket.close()
            me_listening_socket = None


def send_channel_func():
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
    msg = ('$LUPWM,' + str(r1) + ',' + str(r2) + ',' + str(t1) + ',' + str(t2)+'\x0a') # 控拖轮
    # msg = ('$LUPWM,' + str(r1) + ',' + str(t1) + '\x0a')
    print(msg)
    me_socket.sendto(msg.encode("utf-8"), (remote_ip, remote_port))

    if me_socket is not None:
        me_socket.close()
        me_socket = None




if __name__ == "__main__":

    # X = listen_channel_func()
    listen_thread = Thread(target=listen_channel_func)
    listen_thread.start()

    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

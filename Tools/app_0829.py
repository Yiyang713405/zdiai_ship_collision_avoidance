# -*- coding:utf-8 -*-
# 导入相关的库
import ip_controller
import DZL_class
import bezier
import dzl_heading_planning
import rotate
import traceback
import Calculate_collision_avoidance_waypoint2
import Calculate_collision_avoidance_waypoint
import math
import logging
import json

# 配置日志记录器
# logging.basicConfig(filename='app.log',level=logging.DEBUG,encoding='utf-8')
# logger=logging.getLogger(__name__)

# 配置日志格式和编码
log_format = '%(asctime)s - %(levelname)s - %(message)s'
date_format = '%Y-%m-%d %H:%M:%S'
encoding = 'utf-8'

# 创建一个日志记录器
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

# 创建一个日志处理器，将日志输出到指定的文件
file_handler = logging.FileHandler('app.log', encoding=encoding)
file_handler.setFormatter(logging.Formatter(fmt=log_format, datefmt=date_format))

# 将日志处理器添加到日志记录器
logger.addHandler(file_handler)

# logger.info('这是一条警告信息')

# Flask
from flask import Flask, jsonify, request
from flask_compress import Compress  # just compress

app = Flask(__name__)
Compress(app)  # just compress
app.secret_key = 'key'
app.config['JSON_AS_ASCII'] = False
app.config['JSONIFY_MIMETYPE'] = 'application/json;charset=utf-8'


def calculate_direction_angle(X1, Y1, X2, Y2):
    vec_AB = (X2 - X1, Y2 - Y1)

    angle_rad = math.atan2(vec_AB[0], vec_AB[1])
    angle_deg = math.degrees(angle_rad)

    if angle_deg < 0:
        angle_deg += 360

    return angle_deg


# 存储不同场景下不同船的路径的字典
path_cmd_dict = {}


# 开一个接口,接收船舶船首向，东向位移距离，北向位移距离，返回当前船舶的期望舵角(路径固定)
@app.route('/get_rudder', methods=["GET", "POST"])
def get_rudder():
    try:
        scene_id = request.json.get('sceneId')
        ships = request.json.get('ships')
        if not isinstance(ships, list):
            return jsonify(code=500, msg="wrong ships format", data="")
        for ship in ships:
            if "shipId" not in ship:
                return jsonify(code=500, msg="ship id is missing", data="")

        rudder_data = []
        for ship in ships:

            ship_id = ship['shipId']
            logger.info('shipId:' + str(ship_id))

            # 传进来的参数
            # 需要输入本船和他船的状态信息
            cog_os = float(ship['cog_os'])  # 本船航向
            sog_os = float(ship['sog_os'])  # 本船速度
            x_os = float(ship['x_os'])  # 本船位置x
            y_os = float(ship['y_os'])  # 本船位置y

            tships = ship['ship_ts']
            cog_ts = [float(tship['cog_ts']) for tship in tships]
            sog_ts = [float(tship['sog_ts']) for tship in tships]
            x_ts = [float(tship['x_ts']) for tship in tships]
            y_ts = [float(tship['y_ts']) for tship in tships]

            heading = cog_os
            NED_e = x_os
            NED_n = y_os

            # 固定的路径
            path_cmd = DZL_class.Path()
            path_init = bezier.path_points()
            path_init = rotate.rotate_points(path_init, heading)
            path_cmd.add_point(path_init)

            ####新增日志打印
            input_msg = json.dumps({
                "cog_os": cog_os,
                "sog_os": sog_os,
                "x_os": x_os,
                "y_os": y_os,
                "cog_os": cog_ts,
                "sog_os": sog_ts,
                "x_ts": x_ts,
                "y_ts": y_ts
            })
            logger.info("input:\n" + input_msg)

            ######## 规划模块输出——>>控制模块输入：
            # 获取Collision_avoidance_waypoint
            if len(tships) != 0:
                Collision_avoidance_waypoint = Calculate_collision_avoidance_waypoint2.WaypointGet(cog_os, sog_os, x_os,
                                                                                                   y_os, cog_ts, sog_ts,
                                                                                                   x_ts, y_ts)
                # print(Collision_avoidance_waypoint)
            else:
                Collision_avoidance_waypoint = []

            ###新增日志打印
            logger.info("wayPoint:\n" + str(Collision_avoidance_waypoint))

            #################输入为当前船舶艏向、坐标、目标路径点序列；输出为期望航向####################

            # 如果调用下面规控算法，请将Collision_avoidance_waypoint获取函数改为 Calculate_collision_avoidance_waypoint

            # # 如果不存在碰撞风险
            # if Collision_avoidance_waypoint == []:
            #     logger.info('不存在碰撞危险')
            #     target_heading = dzl_heading_planning.LOS().solve(50, heading, NED_e, NED_n, path_cmd)
            #
            # # 如果存在碰撞风险，采用避碰建议航向 else: logger.info('存在避碰危险，触发避碰算法...') target_heading = calculate_direction_angle(
            # x_os, y_os, Collision_avoidance_waypoint[0], Collision_avoidance_waypoint[1])




            #  如果调用下面规控算法，请将Collision_avoidance_waypoint获取函数改为 Calculate_collision_avoidance_waypoint2
            #  Calculate_collision_avoidance_waypoint 和 Calculate_collision_avoidance_waypoint2算法逻辑一样，唯一的不同在于:Calculate_collision_avoidance_waypoint中，当没有碰撞风险时，waypoint==[],此时交由控制去追踪既定轨迹的航向。
            #  而Calculate_collision_avoidance_waypoint2中，当没有碰撞风险时，此时waypoint输出为本船当前航向前1海里的路径点,随后本船就会朝着这个没有碰撞风险的航向航行。
            target_heading = calculate_direction_angle(x_os, y_os, Collision_avoidance_waypoint[0],
                                                       Collision_avoidance_waypoint[1])

            s_psi_cmd_deg = target_heading
            if s_psi_cmd_deg > 360:
                s_psi_cmd_deg = s_psi_cmd_deg - 360
            if s_psi_cmd_deg < heading - 180:
                s_psi_cmd_deg = s_psi_cmd_deg + 360
            if heading < s_psi_cmd_deg - 180:
                s_psi_cmd_deg = s_psi_cmd_deg - 360
            target_angle = s_psi_cmd_deg

            ########  控制模块输出——>>船舶操纵模型（虚拟）/舵机（实船）：
            order_rudderangle = ip_controller.PID(kp=0.5, ki=0.2, kd=0.02, output_min=-30, output_max=30).solve(heading,
                                                                                                                target_angle)

            rudder_data.append({'shipId': ship_id, 'rudderAngle': order_rudderangle, 'target_heading': target_heading})

            logger.info('\n')

        res_data = {'sceneId': scene_id, 'ships': rudder_data}

        return jsonify(code=200, msg="", data=res_data)
    except:
        error_msg = traceback.format_exc()
        print(error_msg)
        return jsonify(code=500, msg=error_msg, data="")


if __name__ == '__main__':
    app.run(host="127.0.0.1",
            debug=True,
            port=8082,
            )

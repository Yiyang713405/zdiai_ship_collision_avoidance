# --------------------------------------------------------------------------- #
# Import
# --------------------------------------------------------------------------- #
import math
import numpy as np
from matplotlib import pyplot as plt


class Point_track(object):
    """ 以追踪点为目标进行导航，返回期望航向、期望航速 """

    def solve(self, tug_point, track_point, h):  #
        tp_x = track_point[0]
        tp_y = track_point[1]
        X_tp_x = tp_x - tug_point[0]
        X_tp_y = tp_y - tug_point[1]

        # tp_pre_x = track_point_pre[0]
        # tp_pre_y = track_point_pre[1]
        # X_tp_pre_x = tp_pre_x - X[2,0]
        # X_tp_pre_y = tp_pre_y - X[3,0]

        in_angle = np.arctan2(X_tp_x, X_tp_y) * (180 / np.pi)
        # in_angle = 450 - in_angle
        tp_angle = in_angle
        if in_angle <= 0:
            tp_angle = in_angle + 360
        elif in_angle > 360:
            tp_angle = in_angle - 360

        # if 90 <= in_angle <= 180:
        #     tp_angle = 450 - in_angle
        # else:
        #     tp_angle = 90 - in_angle

        psi_tp_deg_order = tp_angle

        # Evaluate the direction of turning
        if tug_point[2] >= 0:
            psi_cmd_deg = psi_tp_deg_order

            diff_psi_deg = psi_tp_deg_order - tug_point[2]
            if (180 <= diff_psi_deg) and (diff_psi_deg <= 360):
                psi_cmd_deg = psi_tp_deg_order - 360

            if (-360 <= diff_psi_deg) and (diff_psi_deg <= -180):
                psi_cmd_deg = psi_tp_deg_order + 360

        if tug_point[2] < 0:
            psi_cmd_deg = psi_tp_deg_order - 360

            diff_psi_deg = psi_tp_deg_order - tug_point[2]
            if (0 <= diff_psi_deg) and (diff_psi_deg <= 180):
                psi_cmd_deg = psi_tp_deg_order

            if (540 <= diff_psi_deg) and (diff_psi_deg <= 1080):
                psi_cmd_deg = psi_tp_deg_order - 720

        tp_d = np.sqrt(np.square(X_tp_x) + np.square(X_tp_y))

        u_cmd = tp_d / h

        return psi_cmd_deg, u_cmd

    def __str__(self):
        return '\n'.join(['%s: %s' % item for item in self.__dict__.items()])



class LVS(object):
    """
    A logical virtual ship(lvs) guidance for trajectory tracking.
    """

    def __init__(self, Path, R_receive, virual_u, R_bezier_curve, n, kze, kpsie, km):

        if virual_u is None:
            raise ValueError("没有虚拟小船速度 virual_u.")
        if R_bezier_curve is None:
            raise ValueError("没有贝塞尔曲线半径 R_bezier_curve.")
        if Path is None:
            raise ValueError("没有预设轨迹点 Path.")
        if n is None:
            raise ValueError("没有导航时间 n.")
        if kze is None:
            raise ValueError("没有距离误差参数 k_ze.")
        if kpsie is None:
            raise ValueError("没有角度误差参数 k_psie.")
        if km is None:
            raise ValueError("没有距离参数 km.")
        if R_receive is None:
            raise ValueError("没有动态导航小船接收半径 R_receive.")

        self.virual_u = virual_u
        self.R_receive = R_receive
        self.Path = self.create_smooth_path_v4(Path, R_bezier_curve)
        self.positions = []
        self.velocities = []
        self.lvs_path_cmd(n)
        self.distance = 0
        self.x0 = 0  # 初始化真实船舶位置
        self.y0 = 0
        self.time = 0  # 初始化时间
        self.psi_r, self.last_psi_r = 0, 0
        self.kze = kze
        self.kpsie = kpsie
        self.km = km

    def quadratic_bezier_curve(self, p0, p1, p2, num_points=10):
        """ 生成二阶贝塞尔曲线上的点 """
        t = np.linspace(0, 1, num_points)
        curve = np.array([(1 - s) ** 2 * p0 + 2 * (1 - s) * s * p1 + s ** 2 * p2 for s in t])
        return curve

    def line_circle_intersection(self, circle_center, radius, line_start, line_end):
        """Find intersection points of a circle and a line segment."""

        def sqr(x):
            return x * x

        def norm(a, b):
            return np.sqrt(sqr(a[0] - b[0]) + sqr(a[1] - b[1]))

        # Convert to numpy arrays for vectorized operations
        circle_center = np.array(circle_center)
        line_start = np.array(line_start)
        line_end = np.array(line_end)

        # Calculate the coefficients of the quadratic equation
        a = sqr(line_end[0] - line_start[0]) + sqr(line_end[1] - line_start[1])
        b = 2 * ((line_end[0] - line_start[0]) * (line_start[0] - circle_center[0]) +
                 (line_end[1] - line_start[1]) * (line_start[1] - circle_center[1]))
        c = sqr(circle_center[0]) + sqr(circle_center[1]) + sqr(line_start[0]) + sqr(line_start[1]) - \
            2 * (circle_center[0] * line_start[0] + circle_center[1] * line_start[1]) - \
            sqr(radius)
        # Calculate the discriminant
        d = sqr(b) - 4 * a * c
        # No intersection if the discriminant is negative
        if d < 0:
            return None
        # Two intersections
        t1 = (-b + np.sqrt(d)) / (2 * a)
        t2 = (-b - np.sqrt(d)) / (2 * a)
        # Find the closest intersection point(s)
        intersections = []
        for t in [t1, t2]:
            if 0 <= t <= 1:  # Check if intersection is on the line segment
                intersection = line_start + t * (line_end - line_start)
                intersections.append(intersection)
        # If there are intersections, choose the one closest to the line_start
        if intersections:
            return min(intersections, key=lambda x: norm(x, line_start))
        else:
            return None

    def create_smooth_path_v4(self, path_points, R_bezier_curve):
        """ Create a smooth path by replacing corners with Bezier curve points. """
        smooth_path = [path_points[0]]  # Start with the first point

        for i in range(1, len(path_points) - 1):
            p0 = path_points[i - 1]
            p1 = path_points[i]
            p2 = path_points[i + 1]
            # Calculate intersections for the segments [p0, p1] and [p1, p2]
            intersection1 = self.line_circle_intersection(p1, R_bezier_curve, p0, p1)
            intersection2 = self.line_circle_intersection(p1, R_bezier_curve, p1, p2)

            if intersection1 is not None and intersection2 is not None:
                # Convert intersections to numpy arrays
                intersection1 = np.array(intersection1)
                intersection2 = np.array(intersection2)
                # Generate Bezier curve points
                bezier_curve = self.quadratic_bezier_curve(intersection1, np.array(p1), intersection2)
                smooth_path.extend(bezier_curve)  # Add Bezier points to the path
            else:
                # If no intersection, just add the corner point
                smooth_path.append(p1)
        smooth_path.append(path_points[-1])  # Add the last point
        smooth_path_formatted = [(float(point[0]), float(point[1])) for point in smooth_path]
        return smooth_path_formatted

    def lvs_path_cmd(self, n):
        # 存储每个段落的距离和时间
        distances = []
        times = []
        total_distance = 0
        # 计算每段距离和时间
        for i in range(len(self.Path) - 1):
            dx = self.Path[i + 1][0] - self.Path[i][0]
            dy = self.Path[i + 1][1] - self.Path[i][1]
            distance = math.sqrt(dx ** 2 + dy ** 2)
            total_distance += distance
            distances.append(distance)
            times.append(distance / self.virual_u)
            # print(self.velocities)
            self.velocities.append((self.virual_u * dx / distance, self.virual_u * dy / distance))
        # 遍历每个时间点
        for t in range(n + 1):
            # 确定在哪两个点之间
            time_elapsed = 0
            i = 0
            while i < len(times) and time_elapsed + times[i] < t:
                time_elapsed += times[i]
                i += 1
            # 计算位置
            if i < len(self.Path) - 1:
                # 在路径点之间
                time_in_segment = t - time_elapsed
                ratio = time_in_segment / times[i]
                x = self.Path[i][0] + ratio * (self.Path[i + 1][0] - self.Path[i][0])
                y = self.Path[i][1] + ratio * (self.Path[i + 1][1] - self.Path[i][1])
                u_x, u_y = self.velocities[i]
            else:
                # 如果超出最后一个点，按照最后一个方向继续行进
                dx = self.Path[-1][0] - self.Path[-2][0]
                dy = self.Path[-1][1] - self.Path[-2][1]
                distance_last_segment = math.sqrt(dx ** 2 + dy ** 2)
                time_over = t - time_elapsed
                x = self.Path[-1][0] + time_over * self.virual_u * dx / distance_last_segment
                y = self.Path[-1][1] + time_over * self.virual_u * dy / distance_last_segment
                u_x, u_y = self.velocities[-1]
            self.positions.append([x, y, t, u_x, u_y])
        return self.positions

    def lvs_distance(self):
        my_position = (self.x0, self.y0)
        target_point = next((pos for pos in self.positions if pos[2] == self.time), None)
        if not target_point:
            return "No target point found for the given time."
        # 计算距离
        distance = math.sqrt((my_position[0] - target_point[0]) ** 2 + (my_position[1] - target_point[1]) ** 2)
        return distance

    def LVS(self, time, x0, y0):
        self.time = time  # 这里减1可以实现预瞄
        self.x0 = x0
        self.y0 = y0
        self.last_psi_r = self.psi_r
        distance_now = self.lvs_distance()
        # 检查是否找到了当前和上一个目标点
        current_target = next((pos for pos in self.positions if pos[2] == self.time), None)
        previous_target = next((pos for pos in self.positions if pos[2] == self.time - 1), None)
        if not current_target:
            return None, "No current target point found for the given time."
        if distance_now < self.R_receive and not previous_target:
            return None, "No previous target point found for the given time."
        # 根据是否在接受圆内计算期望方向
        if distance_now < self.R_receive:
            angle_rad = math.atan2((current_target[0] - previous_target[0]),
                                   (current_target[1] - previous_target[1]))
        else:
            angle_rad = math.atan2((current_target[0] - x0), (current_target[1] - y0))
        angle_deg = math.degrees(angle_rad)
        if angle_deg < 0:
            angle_deg += 360
        self.psi_r = angle_deg
        exp_psi = self.psi_r
        exp_u = self.kze * (distance_now - self.km)
        return exp_psi, exp_u

    def reture_path(self):
        return self.positions

    def LVS_test(self, time, x0, y0, psi_now, v_now):
        self.time = time
        self.x0 = x0
        self.y0 = y0
        self.last_psi_r = self.psi_r
        distance_now = self.lvs_distance()
        # 检查是否找到了当前和上一个目标点
        current_target = next((pos for pos in self.positions if pos[2] == self.time), None)
        previous_target = next((pos for pos in self.positions if pos[2] == self.time - 1), None)
        if not current_target:
            return None, "No current target point found for the given time."
        if distance_now < self.R_receive and not previous_target:
            return None, "No previous target point found for the given time."
        # 根据是否在接受圆内计算期望方向
        if distance_now < self.R_receive:
            angle_rad = math.atan2((current_target[1] - previous_target[1]),
                                   (current_target[0] - previous_target[0]))
        else:
            angle_rad = math.atan2((current_target[1] - y0), (current_target[0] - x0))
        angle_deg = math.degrees(angle_rad)
        if angle_deg < 0:
            angle_deg += 360
        self.psi_r = angle_deg  # psi_r 是 论文中/psi_r
        psi_error = self.psi_error_cal(psi_now)

        for position in self.positions:
            if position[2] == time:  # position[2] 是 t
                ud_x = position[3]
                ud_y = position[4]
            else:
                ud_y = 0
                ud_x = 0

        exp_u = (self.kze * (distance_now - self.km) + math.sin(math.radians(self.psi_r)) * ud_y \
                 + math.cos(math.radians(self.psi_r)) * ud_x - v_now * math.sin(math.radians(psi_error))) / \
                (math.cos(math.radians(psi_error)))
        # exp_u = (self.kze * (distance_now - self.km) + math.sin(math.radians(self.psi_r)) * ud_y\
        #         + math.cos(math.radians(self.psi_r)) * ud_x - v_now * math.sin(math.radians(psi_error)))

        if self.psi_r - self.last_psi_r > 180:
            psi_r_dot = self.psi_r - psi_now - 360
        elif self.psi_r - self.last_psi_r < 180:
            psi_r_dot = self.psi_r - psi_now + 360
        else:
            psi_r_dot = self.psi_r - self.last_psi_r

        exp_r = psi_error * self.kpsie + psi_r_dot
        return exp_u, exp_r





        # if intersections:
        #     return min(intersections, key=lambda x: norm(x, line_start))
        # else:
        #     return None


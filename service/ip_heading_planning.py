# -*- coding: utf-8 -*-
"""
ip_heading_planning defines functions for heading plans based on provide path points.
A typical method for heading planning is Line-of-Sight (LOS).
"""
# --------------------------------------------------------------------------- #
# Import
# --------------------------------------------------------------------------- #
import numpy as np
# --------------------------------------------------------------------------- #
# Documentation
# --------------------------------------------------------------------------- #
__author__ = "Jialun Liu"
__date__ = "2020/3/1"
__email__ = "jialunliu@outlook.com"
__copyright__ = "Copyright (c) 2020 Jialun Liu. All rights reserved."
# --------------------------------------------------------------------------- #
# Function
# --------------------------------------------------------------------------- #


class LOS(object):
    """
    LOS uses the Light of Sight (LOS) method to perform path following.
    """

    def __init__(self, R2LPP=2.0):
        self.R2LPP = R2LPP  # (1) Ratio of the radius of the accept circle to LPP接受圆半径与LPP的比值

    def solve(self, vessel, path):

        accept_radius = self.R2LPP * vessel.hull.LPP

        # psi_cmd_deg is assigned to psi_deg in case that no further LOS path_point is found.
        # 在没有找到其他LOS路径点的情况下，psi_cmd_deg被分配给psi_deg。
        psi_cmd_deg = vessel.psi_deg  # (deg) Heading angle command

        ii = path.count
        jj = path.num
        if ii < jj - 2:
            path_point_0 = path.coord[path.count]
            path_point_1 = path.coord[path.count + 1]
            path_point_2 = path.coord[path.count + 2]
            # Check whether the ship enters the circle of the next path path_point based on
            # trajectory differences. Accordingly, calculate the heading angle to control:
            distance_detection = np.hypot(vessel.e - path_point_1[1], vessel.n - path_point_1[0])
            if distance_detection < accept_radius:
                # If the ship has entered the circle of the next path path_point
                p1x = path_point_1[0]
                p1y = path_point_1[1]
                p2x = path_point_2[0]
                p2y = path_point_2[1]
                path.count += 1
            else:
                p1x = path_point_0[0]
                p1y = path_point_0[1]
                p2x = path_point_1[0]
                p2y = path_point_1[1]

            # Set a dynamic circle around the ship
            at = np.hypot(vessel.e - p1y, vessel.n - p1x)  # 当前位置到path_point_1的距离
            bt = np.hypot(p2y - vessel.e, p2x - vessel.n)  # 当前位置到path_point_2的距离
            ct = np.hypot(p2y - p1y, p2x - p1x)  # path_point_1到path_point_2的距离
            # radius_min = np.sqrt(at ** 2 - ((ct ** 2 - bt ** 2 + at ** 2) / (2 * ct)) ** 2)

            # Radius of the circle around the ship
            # ship_circle_radius = radius_min + 2 * vessel.hull.LPP
            ship_circle_radius = 2 * vessel.hull.LPP

            # Solve LOS equations
            if (p2y - p1y) == 0 and (p2x - p1x) > 0:
                # If the denominator分母 is 0
                y_0 = p2y
                x_0 = vessel.n + np.sqrt(ship_circle_radius ** 2 - (vessel.e - p2y) ** 2)
                y_1 = p2y
                x_1 = vessel.n + np.sqrt(ship_circle_radius ** 2 - (vessel.e - p2y) ** 2)
            else:
                if (p2y - p1y) == 0 and (p2x - p1x) < 0:
                    # If the denominator is 0
                    y_0 = p2y
                    x_0 = vessel.n - np.sqrt(ship_circle_radius ** 2 - (vessel.e - p2y) ** 2)
                    y_1 = p2y
                    x_1 = vessel.n - np.sqrt(ship_circle_radius ** 2 - (vessel.e - p2y) ** 2)
                else:
                    # Normal conditions
                    d = (p2x - p1x) / (p2y - p1y)  # p1和p2连线的斜率
                    g = p1x - d * p1y
                    a = (1 + d ** 2)
                    b = 2 * (d * g - d * vessel.n - vessel.e)
                    c = vessel.e ** 2 + vessel.n ** 2 + g ** 2 - ship_circle_radius ** 2 - 2 * g * vessel.n
                    y_0 = (-b + np.sqrt(b ** 2 - 4 * a * c)) / (2 * a)
                    x_0 = p1x + d * (y_0 - p1y)

                    y_1 = (-b - np.sqrt(b ** 2 - 4 * a * c)) / (2 * a)
                    x_1 = p1x + d * (y_1 - p1y)

            if np.isreal(x_0) and np.isreal(x_1) and np.isreal(y_0) and np.isreal(y_1):
                # Check if there is a crossing path_point:
                if np.hypot(p2y - y_0, p2x - x_0) < np.hypot(p2y - y_1, p2x - x_1):
                    # Check which path_point is closer
                    path_los_y = y_0
                    path_los_x = x_0
                else:
                    path_los_y = y_1
                    path_los_x = x_1

                U_los_y = path_los_y - vessel.e
                U_los_x = path_los_x - vessel.n

                in_angle = np.arctan2(U_los_x, U_los_y) * (180 / np.pi)

                if 90 <= in_angle <= 180:
                    LOS_angle = 450 - in_angle
                else:
                    LOS_angle = 90 - in_angle

                # Assign the angle of LOS to the ordered psi
                psi_LOS_deg_order = LOS_angle

            else:
                print('Fail to follow the {path_count}th path with LOS.'.format(path_count=path.count))
                in_angle = np.arctan2((p2x - p1x), (p2y - p1y)) * (180 / np.pi)

                if 90 <= in_angle <= 180:
                    LOS_angle = 450 - in_angle
                else:
                    LOS_angle = 90 - in_angle

                # Assign the angle of LOS to the ordered psi
                psi_LOS_deg_order = vessel.psi_deg + LOS_angle

            # Evaluate the direction of turning
            if vessel.psi_deg >= 0:
                psi_cmd_deg = psi_LOS_deg_order

                diff_psi_deg = psi_LOS_deg_order - vessel.psi_deg
                if (180 <= diff_psi_deg) and (diff_psi_deg <= 360):
                    psi_cmd_deg = psi_LOS_deg_order - 360

                if (-360 <= diff_psi_deg) and (diff_psi_deg <= -180):
                    psi_cmd_deg = psi_LOS_deg_order + 360

            if vessel.psi_deg < 0:
                psi_cmd_deg = psi_LOS_deg_order - 360

                diff_psi_deg = psi_LOS_deg_order - vessel.psi_deg
                if (0 <= diff_psi_deg) and (diff_psi_deg <= 180):
                    psi_cmd_deg = psi_LOS_deg_order

                if (540 <= diff_psi_deg) and (diff_psi_deg <= 1080):
                    psi_cmd_deg = psi_LOS_deg_order - 720

        return psi_cmd_deg

    def __str__(self):
        return '\n'.join(['%s: %s' % item for item in self.__dict__.items()])


class LOS_4m(object):
    """
    LOS uses the Light of Sight (LOS) method to perform path following.
    """

    def __init__(self, R2LPP=1.5):
        self.R2LPP = R2LPP  # (1) Ratio of the radius of the accept circle to LPP

    def solve(self, vessel, path):

        accept_radius = self.R2LPP * vessel.hull.LPP

        # psi_cmd_deg is assigned to psi_deg in case that no further LOS path_point is found.
        psi_cmd_deg = vessel.psi_deg  # (deg) Heading angle command
        # view.path_count = path.count

        if path.count < path.num - 2:

            path_point_0 = path.coord[path.count]
            path_point_1 = path.coord[path.count + 1]
            path_point_2 = path.coord[path.count + 2]

            print("当前航向路径点：", path.count, path_point_0, path_point_1, path_point_2)  # 打印当前路径点
            print("当前平面坐标点：", vessel.e, vessel.n)

            # Check whether the ship enters the circle of the next path path_point based on
            # trajectory differences. Accordingly, calculate the heading angle to control:
            distacne_err = np.hypot(vessel.e - path_point_1[1], vessel.n - path_point_1[0])
            print("误差", distacne_err, accept_radius)

            if distacne_err < accept_radius:

                # If the ship has entered the circle of the next path path_point
                p1x = path_point_1[0]
                p1y = path_point_1[1]
                p2x = path_point_2[0]
                p2y = path_point_2[1]

                path.count += 1

            else:
                p1x = path_point_0[0]
                p1y = path_point_0[1]
                p2x = path_point_1[0]
                p2y = path_point_1[1]

            # Set a dynamic circle around the ship
            at = np.hypot(vessel.e - p1y, vessel.n - p1x)
            bt = np.hypot(p2y - vessel.e, p2x - vessel.n)
            ct = np.hypot(p2y - p1y, p2x - p1x)
            print(at, bt, ct, at ** 2 - ((ct ** 2 - bt ** 2 + at ** 2) / (2 * ct)) ** 2)

            radius_min = np.sqrt(at ** 2 - ((ct ** 2 - bt ** 2 + at ** 2) / (2 * ct)) ** 2)

            ############保存los误差##############
            file = open("radius_min", 'a+')
            file.write(str(radius_min) + ',')
            file.close()

            # Radius of the circle around the ship
            ship_circle_radius = radius_min + 3 * vessel.hull.LPP

            # Solve LOS equations
            if (p2y - p1y) == 0 and (p2x - p1x) > 0:
                # If the denominator is 0
                y_0 = p2y
                x_0 = vessel.n + np.sqrt(ship_circle_radius ** 2 - (vessel.e - p2y) ** 2)

                y_1 = p2y
                x_1 = vessel.n + np.sqrt(ship_circle_radius ** 2 - (vessel.e - p2y) ** 2)
            else:

                if (p2y - p1y) == 0 and (p2x - p1x) < 0:
                    # If the denominator is 0
                    y_0 = p2y
                    x_0 = vessel.n - np.sqrt(ship_circle_radius ** 2 - (vessel.e - p2y) ** 2)
                    y_1 = p2y
                    x_1 = vessel.n - np.sqrt(ship_circle_radius ** 2 - (vessel.e - p2y) ** 2)
                else:
                    # Normal conditions
                    d = (p2x - p1x) / (p2y - p1y)
                    g = p1x - d * p1y
                    a = (1 + d ** 2)
                    b = 2 * (d * g - d * vessel.n - vessel.e)
                    c = vessel.e ** 2 + vessel.n ** 2 + g ** 2 - ship_circle_radius ** 2 - 2 * g * vessel.n
                    y_0 = (-b + np.sqrt(b ** 2 - 4 * a * c)) / (2 * a)
                    x_0 = p1x + d * (y_0 - p1y)

                    y_1 = (-b - np.sqrt(b ** 2 - 4 * a * c)) / (2 * a)
                    x_1 = p1x + d * (y_1 - p1y)

            if np.isreal(x_0) and np.isreal(x_1) and np.isreal(y_0) and np.isreal(y_1):
                # Check if there is a crossing path_point:
                if np.hypot(p2y - y_0, p2x - x_0) < np.hypot(p2y - y_1, p2x - x_1):
                    # Check which path_point is closer
                    path_los_y = y_0
                    path_los_x = x_0
                else:
                    path_los_y = y_1
                    path_los_x = x_1

                U_los_y = path_los_y - vessel.e
                U_los_x = path_los_x - vessel.n

                in_angle = np.arctan2(U_los_x, U_los_y) * (180 / np.pi)

                if 90 <= in_angle <= 180:
                    LOS_angle = 450 - in_angle
                else:
                    LOS_angle = 90 - in_angle

                # Assign the angle of LOS to the ordered psi
                psi_LOS_deg_order = LOS_angle
                print("LOS_angle", LOS_angle)

            else:
                print('Fail to follow the {path_count}th path with LOS.'.format(path_count=path.count))
                in_angle = np.arctan2((p2x - p1x), (p2y - p1y)) * (180 / np.pi)

                if 90 <= in_angle <= 180:
                    LOS_angle = 450 - in_angle
                else:
                    LOS_angle = 90 - in_angle

                # Assign the angle of LOS to the ordered psi
                psi_LOS_deg_order = vessel.psi_deg + LOS_angle

            # # Evaluate the direction of turning 判断 正负180度角，需要关掉。
            # if vessel.psi_deg >= 0:
            #     psi_cmd_deg = psi_LOS_deg_order
            #
            #     diff_psi_deg = psi_LOS_deg_order - vessel.psi_deg
            #     if (180 <= diff_psi_deg) and (diff_psi_deg <= 360):
            #         psi_cmd_deg = psi_LOS_deg_order - 360
            #
            #     if (-360 <= diff_psi_deg) and (diff_psi_deg <= -180):
            #         psi_cmd_deg = psi_LOS_deg_order + 360
            #
            # if vessel.psi_deg < 0:
            #     psi_cmd_deg = psi_LOS_deg_order - 360
            #
            #     diff_psi_deg = psi_LOS_deg_order - vessel.psi_deg
            #     if (0 <= diff_psi_deg) and (diff_psi_deg <= 180):
            #         psi_cmd_deg = psi_LOS_deg_order
            #
            #     if (540 <= diff_psi_deg) and (diff_psi_deg <= 1080):
            #         psi_cmd_deg = psi_LOS_deg_order - 720
        else:
            print("reach the destination")
            path.count = 0
            psi_LOS_deg_order = vessel.psi_deg

        if np.isnan(psi_LOS_deg_order):
            psi_LOS_deg_order= vessel.psi_deg

        return psi_LOS_deg_order

    def __str__(self):
        return '\n'.join(['%s: %s' % item for item in self.__dict__.items()])

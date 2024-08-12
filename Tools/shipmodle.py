# import matplotlib.pyplot as plt
import matplotlib.path as mpath
from matplotlib.markers import MarkerStyle
from matplotlib import transforms
import math


def calculate_distance(x1, y1, x2, y2):
    """
    两点距离计算
    :param x1:
    :param y1:
    :param x2:
    :param y2:
    :return: 两点距离
    """
    # 计算水平距离和垂直距离
    horizontal_distance = abs(x2 - x1)
    vertical_distance = abs(y2 - y1)

    # 计算斜边距离（两点之间的直线距离）
    distance = math.sqrt(horizontal_distance ** 2 + vertical_distance ** 2)

    return distance


def shipmodle(cog_ts, x_ts, y_ts, cog_os, x_os, y_os):
    """
    绘制船舶形状标图标
    :param cog_ts: 目标船航速
    :param x_ts: 目标船x坐标
    :param y_ts: 目标船y坐标
    :param cog_os: 本船航速
    :param x_os: 本船x坐标
    :param y_os: 本船y坐标
    :return: 所有船的形状图标,有风险船舶的形状图标
    """
    icon_all = []
    icon_risk = []
    icon_cpa = []
    x_os = x_os / 1852
    y_os = y_os / 1852
    x_ts = [element / 1852 for element in x_ts]
    y_ts = [element / 1852 for element in y_ts]
    x_ts.append(x_os)
    y_ts.append(y_os)
    cog_ts.append(cog_os)
    # 添加一个路径path，路径的详细解释后面会讲到，相比于简单的patch，稍显复杂
    # -------------------------------------------------------------------------------------------------------
    # 识别5海里范围内的船
    x_recognizable = []  # 出现在本船一定范围内的可以识别的目标船的x坐标
    y_recognizable = []  # 出现在本船一定范围内的可以识别的目标船的y坐标
    cog_recognizable = []  # 出现在本船一定范围内的可以识别的目标船的航向
    for i in range(len(x_ts)-1):
        distance = calculate_distance(x_ts[i], y_ts[i], x_os, y_os)
        if distance < 5:
            x_recognizable.append(x_ts[i])
            y_recognizable.append(y_ts[i])
            cog_recognizable.append(cog_ts[i])
    # -------------------------------------------------------------------------------------------------------

    class UnsizedMarker(MarkerStyle):
        def _set_custom_marker(self, path):
            self._transform = transforms.IdentityTransform()
            self._path = path
    for i in range(len(x_ts)):
        Path = mpath.Path
        ship_all = Path([[x_ts[i] - 1, y_ts[i]], [x_ts[i] - 1, y_ts[i] + 3],
                     [x_ts[i], y_ts[i] + 6], [x_ts[i] + 1, y_ts[i] + 3],
                     [x_ts[i] + 1, y_ts[i]], [x_ts[i], y_ts[i]]],
                    [Path.MOVETO, Path.LINETO, Path.CURVE3, Path.CURVE3, Path.LINETO, Path.CLOSEPOLY])
        m1 = UnsizedMarker(ship_all)
        R = transforms.Affine2D().rotate_deg(-cog_ts[i])
        ship_all_1 = ship_all.transformed(R)
        m2 = UnsizedMarker(ship_all_1)
        icon_all.append(m2)
    for i in range(len(x_recognizable)):
        Path = mpath.Path
        ship_risk = Path([[x_recognizable[i] - 1, y_recognizable[i]], [x_recognizable[i] - 1, y_recognizable[i] + 3],
                         [x_recognizable[i], y_recognizable[i] + 6], [x_recognizable[i] + 1, y_recognizable[i] + 3],
                         [x_recognizable[i] + 1, y_recognizable[i]], [x_recognizable[i], y_recognizable[i]]],
                         [Path.MOVETO, Path.LINETO, Path.CURVE3, Path.CURVE3, Path.LINETO, Path.CLOSEPOLY])
        m3 = UnsizedMarker(ship_risk)
        R = transforms.Affine2D().rotate_deg(-cog_recognizable[i])
        ship_risk_1 = ship_risk.transformed(R)
        m4 = UnsizedMarker(ship_risk_1)
        icon_risk.append(m4)
    for i in range(len(x_recognizable)):
        Path = mpath.Path
        ship_cpa = Path([[x_recognizable[i] - 1, y_recognizable[i]], [x_recognizable[i] - 1, y_recognizable[i] + 3],
                         [x_recognizable[i], y_recognizable[i] + 6], [x_recognizable[i] + 1, y_recognizable[i] + 3],
                         [x_recognizable[i] + 1, y_recognizable[i]], [x_recognizable[i], y_recognizable[i]]],
                         [Path.MOVETO, Path.LINETO, Path.CURVE3, Path.CURVE3, Path.LINETO, Path.CLOSEPOLY])
        m5 = UnsizedMarker(ship_cpa)
        R = transforms.Affine2D().rotate_deg(-cog_recognizable[i])
        ship_cpa_1 = ship_cpa.transformed(R)
        m6 = UnsizedMarker(ship_cpa_1)
        icon_cpa.append(m6)
    return icon_all, icon_risk, icon_cpa

# if __name__ == "__main__":
#     lat_ts, lon_ts, cog_ts, sog_ts = AISDataGet.RedisDataGet()
#     x_ts, y_ts = AISDataGet.LatLon2Cartesian(lat_ts, lon_ts)
#     # Lat_os, Lon_os, COG_os, SOG_os = listen_channel_func()
#     # x_os, y_os = LatLon2Cartesian(Lat_os, Lon_os)
#     x_os = x_ts[6]
#     y_os = y_ts[6]
#     cog_os = cog_ts[6]
#     sog_os = sog_ts[6]
#     x_ts = x_ts[0:6]
#     y_ts = y_ts[0:6]
#     cog_ts = cog_ts[0:6]
#     sog_ts = sog_ts[0:6]
#     waypoint, min_index, x_recognizable, y_recognizable = WaypointGet.WaypointGet(cog_os, sog_os, x_os, y_os, cog_ts, sog_ts, x_ts, y_ts)
#     icon_all, icon_risk = shipmodle(cog_ts, x_ts, y_ts, cog_os, x_os, y_os)
#     # 显示
#     plt.figure(dpi=128)
#     plt.grid(True)
#     plt.grid(color='black', linestyle='--', linewidth=0.3, alpha=0.3)
#     plt.xlabel('x')
#     plt.ylabel('y')
#     plt.xlim(-30, 30)
#     plt.ylim(-30, 30)
#     print(len(icon_all))
#     for i in range(len(x_ts)):
#         plt.scatter(x_ts[i]/1852, y_ts[i]/1852, marker=icon_all[i], s=10, facecolor="none", edgecolors="black")
#     # for i in range(len(x_recognizable)):
#     #     plt.scatter(x_recognizable[i]/1852, y_recognizable[i]/1852, marker=icon_risk[i], s=10, facecolor="black", edgecolors="black")
#     plt.show()

# -*- coding: utf-8 -*-
"""
ip_path_planning defines methods for path planning.
"""

# --------------------------------------------------------------------------- #
# Import
# --------------------------------------------------------------------------- #
import matplotlib.pyplot as plt
import numpy as np

# --------------------------------------------------------------------------- #
# Documentation
# --------------------------------------------------------------------------- #
__author__ = "Jialun Liu"
__date__ = "2020/3/1"
__email__ = "jialunliu@outlook.com"
__copyright__ = "Copyright (c) 2020 Jialun Liu. All rights reserved."


# --------------------------------------------------------------------------- #
# Path
# --------------------------------------------------------------------------- #
class Path(object):

    def __init__(self):
        self.count = 0  # Path path_point count
        self.coord = None  # Path coordinates in ECEF coordinate system
        self.num = 0  # Number of path points

    def set_num(self):
        self.num = self.coord.shape[0]

    def pick_point(self, path_slice):
        self.coord = self.coord[path_slice]
        self.set_num()

    def get_curr_point(self):
        return self.coord[self.count][:]

    def get_prev_point(self):
        if self.count == 0:
            print("WARNING! The current path_point is returned as it is the first path_point.")
            return self.coord[self.count][:]
        else:
            return self.coord[self.count - 1][:]

    def get_next_point(self):
        if self.count == self.num - 1:
            print("WARNING! The current path_point is returned as it is the last path_point.")
            return self.coord[self.count][:]
        else:
            return self.coord[self.count + 1][:]

    #
    def add_point(self, path_point):
        if self.coord is None:
            self.coord = np.array(path_point, dtype=float)
        else:
            self.coord = np.row_stack((self.coord, np.array(path_point, dtype=float)))

        self.set_num()

    def reverse_coord(self):
        self.coord = np.flipud(self.coord)

    def reverse_count(self):
        self.count = self.num - self.count - 1

    def __str__(self):
        return '\n'.join(['%s:\t%s' % item for item in self.__dict__.items()])


# --------------------------------------------------------------------------- #
# Artificial Potential Field (APF)
# --------------------------------------------------------------------------- #
class APF(object):

    def __init__(self, grid_size=1.0, area_radius=20.0, safe_radius=10.0, attractive_gain=5.0, repulsive_gain=100.0):
        """
        Artificial Potential Field based path planning method.
        Code reference: https://github.com/AtsushiSakai/PythonRobotics
        Algorithm reference: https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf
        Args:
            grid_size (float): Resolution of the potential rho (m)
            area_radius (float): Potential area radius around the obstacles (m)
            safe_radius ([float]): Radius of the safe margin around own ship (m)
            attractive_gain (float): Attractive potential gain
            repulsive_gain (float): Repulsive potential gain
        Returns:
            potential_field [ndarray]: Total potential of each grid in the potential rho
            field_x_min [float]: Minimum x coordinate in the potential rho
            field_y_min [float]: Minimum y coordinate in the potential rho
        """
        self.grid_size = grid_size
        self.area_radius = area_radius
        self.safe_radius = safe_radius
        self.attractive_gain = attractive_gain
        self.repulsive_gain = repulsive_gain

    def get_potential_field(self, end_point, obstacles):
        """
        Generate a potential rho.
        """
        field_x_min = min(obstacles[:, 0]) - self.area_radius  # 利用最外围障碍物的最大和最小坐标，往外扩展势场半径的范围，获取网格空间
        field_y_min = min(obstacles[:, 1]) - self.area_radius
        field_x_max = max(obstacles[:, 0]) + self.area_radius
        field_y_max = max(obstacles[:, 1]) + self.area_radius

        field_x_range = int(round((field_x_max - field_x_min) / self.grid_size))  # x轴的网格区间数值
        field_y_range = int(round((field_y_max - field_y_min) / self.grid_size))  # y轴的网格区间数值

        # Calculate potential of each point in the rho
        potential_field = np.zeros((field_x_range, field_y_range))

        for i_field_x in range(field_x_range):
            field_x = i_field_x * self.grid_size + field_x_min

            for i_field_y in range(field_y_range):
                field_y = i_field_y * self.grid_size + field_y_min
                attractive_potential = self.get_attractive_potential(field_x, field_y, end_point)
                repulsive_potential = self.get_repulsive_potential(field_x, field_y, obstacles)
                total_potential = attractive_potential + repulsive_potential
                potential_field[i_field_x, i_field_y] = total_potential

        return potential_field, field_x_min, field_y_min

    def get_attractive_potential(self, field_x, field_y, end_point):
        """
        Calculate attractive potential
        """
        return 0.5 * self.attractive_gain * np.hypot(field_x - end_point[0], field_y - end_point[1])

    def get_repulsive_potential(self, field_x, field_y, obstacles):
        """
        Calculate repulsive potential
        """

        # Search for the nearest obstacle
        dist_min_id = -1  # 这就是一个dist_min的数值存储器，可以用任意值初始化
        dist_min = float("inf")  # 同上
        for i_obstacle in range(obstacles.shape[0]):
            dist = np.hypot(field_x - obstacles[i_obstacle, 0], field_y - obstacles[i_obstacle, 1])
            if dist_min >= dist:
                dist_min = dist
                dist_min_id = i_obstacle

        # Calculate repulsive potential
        dist_to_obstacle = np.hypot(field_x - obstacles[dist_min_id, 0], field_y - obstacles[dist_min_id, 1])

        if dist_to_obstacle <= self.safe_radius:
            if dist_to_obstacle <= 0.1:
                dist_to_obstacle = 0.1

            return 0.5 * self.repulsive_gain * (1.0 / dist_to_obstacle - 1.0 / self.safe_radius) ** 2
        else:
            return 0.0

    @staticmethod
    def get_motion_model():
        """
        Provide a motion model of dx, dy with eight point connectivity.
        """
        motion = [[1, 0], [0, 1], [-1, 0], [0, -1], [-1, -1], [-1, 1], [1, -1], [1, 1]]

        return motion

    def solve(self, start_point, end_point, obstacles):
        """
        Artificial Potential Field based path plan method.

        Args:
            start_point (list): Coordinates of the start point
            end_point (list): Coordinates of the end point
            obstacles (ndarray): Coordinates of the obstacles to avoid
        """

        # Calculate potential rho
        potential_map, field_x_min, field_y_min = self.get_potential_field(end_point, obstacles)

        # Initialize variables
        dist = np.hypot(start_point[0] - end_point[0], start_point[1] - end_point[1])

        i_x_start = round((start_point[0] - field_x_min) / self.grid_size)
        i_y_start = round((start_point[1] - field_y_min) / self.grid_size)

        path_plan_x, path_plan_y = [start_point[0]], [start_point[1]]
        field_plan_x, field_plan_y = [i_x_start], [i_y_start]  # 这里是起点的网格位置获取
        motion = self.get_motion_model()

        print('APF starts ...')
        # Search for the shortest path with wavefront method using eight point connectivity
        while dist >= self.grid_size:
            potential_min = float("inf")
            potential_min_x_id, potential_min_y_id = -1, -1
            for i, _ in enumerate(motion):
                inx = int(i_x_start + motion[i][0])
                iny = int(i_y_start + motion[i][1])

                if inx >= len(potential_map) or iny >= len(potential_map[0]):
                    potential = float("inf")  # outside area
                else:
                    potential = potential_map[inx][iny]
                if potential_min > potential:
                    potential_min = potential
                    potential_min_x_id = inx
                    potential_min_y_id = iny

            i_x_start = potential_min_x_id
            i_y_start = potential_min_y_id
            path_point_x = i_x_start * self.grid_size + field_x_min
            path_point_y = i_y_start * self.grid_size + field_y_min

            dist = np.hypot(end_point[0] - path_point_x, end_point[1] - path_point_y)

            path_plan_x.append(path_point_x)
            path_plan_y.append(path_point_y)

            field_plan_x.append(i_x_start)
            field_plan_y.append(i_y_start)

        print('APF ends ... ')

        path_point = np.row_stack((np.array(path_plan_x), np.array(path_plan_y))).T
        field_plan = np.row_stack((np.array(field_plan_x), np.array(field_plan_y))).T

        path_plan = Path()
        path_plan.add_point(path_point)

        return path_plan, field_plan, potential_map


# --------------------------------------------------------------------------- #
# Test APF
# --------------------------------------------------------------------------- #
def test_path_plan_APF():
    grid_size = 0.5  # Resolution of the potential rho (m)
    area_radius = 20.0  # Potential area radius (m)
    safe_radius = 5.0  # Safe marin around own ship in radius (m)
    attractive_gain = 5.0  # Attractive potential gain (-)
    repulsive_gain = 100.0  # Repulsive potential gain (-)
    start_point = [0.0, 0.0]  # Position of start point (m)
    end_point = [35.0, 35.0]  # Position of end point (m)

    # Obstacle positions (m)
    obstacles = np.array([[10.0, 10.0], [15.0, 20.0], [5.0, 15.0], [20.0, 26.0], [25.0, 25.0]])

    test_path_plan = APF(grid_size, area_radius, safe_radius, attractive_gain, repulsive_gain)

    path_plan, field_plan, potential_map = test_path_plan.solve(start_point, end_point, obstacles)

    plt.figure()
    plt.pcolor(np.array(potential_map).T, vmax=100.0, cmap='Blues')
    plt.plot(field_plan[:, 0], field_plan[:, 1], linestyle='--', color='r')
    plt.axis("equal")
    plt.grid(True)
    plt.show()

    plt.figure()
    plt.plot(path_plan.coord[:, 1], path_plan.coord[:, 0], color='g', linestyle='--', label='Planned path')
    plt.plot(
        obstacles[:, 1], obstacles[:, 0], color='r', linestyle='None', marker='s', markersize=8, label='Obstacles')
    plt.plot(
        start_point[0], start_point[1], color='y', linestyle='None', marker='o', markersize=8, label='Start path_point')
    plt.plot(end_point[0], end_point[1], color='m', linestyle='None', marker='o', markersize=8, label='End path_point')

    plt.xlabel(r'$y_0$ (m)')
    plt.ylabel(r'$x_0$ (m)')
    plt.legend(loc='best')
    plt.axis('equal')
    plt.grid(True)

    plt.show()


# --------------------------------------------------------------------------- #
# Astar path planning
# --------------------------------------------------------------------------- #
"""
       Astar search from two side based path planning method.

       Code reference:
       https://github.com/AtsushiSakai/PythonRobotics

       Algorithm reference:
       https://www.cs.cmu.edu/~motionplanning/lecture/AppH-astar-dstar_howie.pdf
"""
show_animation = True


class AStar(object):
    def __init__(self, start, end, top_vertex, bottom_vertex, obstacles):
        self.start = start
        self.end = end
        self.top_vertex = top_vertex
        self.bottom_vertex = bottom_vertex
        self.obstacles = obstacles

    class Node:
        """node with properties of g, h, coordinate and parent node"""

        def __init__(self, G=0, H=0, coordinate=None, parent=None):
            self.G = G
            self.H = H
            self.F = G + H
            self.parent = parent
            self.coordinate = coordinate

        def reset_f(self):
            self.F = self.G + self.H

    def hcost(self, node_coordinate, goal):
        dx = abs(node_coordinate[0] - goal[0])
        dy = abs(node_coordinate[1] - goal[1])
        hcost = dx + dy
        return hcost

    def gcost(self, fixed_node, update_node_coordinate):
        dx = abs(fixed_node.coordinate[0] - update_node_coordinate[0])
        dy = abs(fixed_node.coordinate[1] - update_node_coordinate[1])
        gc = np.hypot(dx, dy)  # gc = move from fixed_node to update_node
        gcost = fixed_node.G + gc  # gcost = move from start point to update_node
        return gcost

    def boundary_and_obstacles(self):  # start, goal, top_vertex, bottom_vertex, obstacles):
        """
        Args:
             start: start coordinate
             goal: goal coordinate
             top_vertex: top right vertex coordinate of boundary
             bottom_vertex: bottom left vertex coordinate of boundary
             obs_number: number of obstacles generated in the map
        return: boundary_obstacle array, obstacle list
        """
        # below can be merged into a rectangle boundary
        ay = list(range(self.bottom_vertex[1], self.top_vertex[1]))
        ax = [self.bottom_vertex[0]] * len(ay)
        cy = ay
        cx = [self.top_vertex[0]] * len(cy)
        bx = list(range(self.bottom_vertex[0] + 1, self.top_vertex[0]))
        by = [self.bottom_vertex[1]] * len(bx)
        dx = [self.bottom_vertex[0]] + bx + [self.top_vertex[0]]
        dy = [self.top_vertex[1]] * len(dx)

        # generate random obstacles
        # x y coordinate in certain order for boundary
        x = ax + bx + cx + dx
        y = ay + by + cy + dy

        # remove start and goal coordinate in obstacle list
        obs = self.obstacles.T.tolist()
        obstacle = [coor for coor in obs if coor != self.start and coor != self.end]
        obs_array = np.array(obstacle)
        bound = np.vstack((x, y)).T
        bound_obs = np.vstack((bound, obs_array))
        obstacle_exp = []
        for coor in obstacle:
            for x in range(coor[0] - 2, coor[0] + 3):
                for y in range(coor[1] - 2, coor[1] + 3):
                    if [x, y] not in obstacle:
                        obstacle_exp.append([x, y])
        obstacle_expand = []
        for i in obstacle_exp:
            if i not in obstacle_expand:
                obstacle_expand.append(i)
        obs_expand_array = np.array(obstacle_expand)
        obs_and_expand = obstacle_expand + obstacle
        obs_and_expand_array = np.array(obs_and_expand)
        bound_obs_and_expand_array = np.vstack((bound, obs_and_expand_array))
        return bound_obs, obstacle, obs_expand_array, bound_obs_and_expand_array

    def find_neighbor(self, node, bound_obs_and_expand_array, closed):
        # generate neighbors in certain condition
        ob_list = bound_obs_and_expand_array.tolist()
        neighbor: list = []
        for x in range(node.coordinate[0] - 1, node.coordinate[0] + 2):
            for y in range(node.coordinate[1] - 1, node.coordinate[1] + 2):
                if [x, y] not in ob_list:
                    # find all possible neighbor nodes
                    neighbor.append([x, y])
        # remove node violate the motion rule
        # 1. remove node.coordinate itself
        neighbor.remove(node.coordinate)
        # 2. remove neighbor nodes who cross through two diagonal
        # positioned obstacles since there is no enough space for
        # robot to go through two diagonal positioned obstacles

        # top bottom left right neighbors of node
        top_nei = [node.coordinate[0], node.coordinate[1] + 1]
        bottom_nei = [node.coordinate[0], node.coordinate[1] - 1]
        left_nei = [node.coordinate[0] - 1, node.coordinate[1]]
        right_nei = [node.coordinate[0] + 1, node.coordinate[1]]
        # neighbors in four vertex
        lt_nei = [node.coordinate[0] - 1, node.coordinate[1] + 1]
        rt_nei = [node.coordinate[0] + 1, node.coordinate[1] + 1]
        lb_nei = [node.coordinate[0] - 1, node.coordinate[1] - 1]
        rb_nei = [node.coordinate[0] + 1, node.coordinate[1] - 1]

        # remove the unnecessary neighbors
        if top_nei and left_nei in ob_list and lt_nei in neighbor:
            neighbor.remove(lt_nei)
        if top_nei and right_nei in ob_list and rt_nei in neighbor:
            neighbor.remove(rt_nei)
        if bottom_nei and left_nei in ob_list and lb_nei in neighbor:
            neighbor.remove(lb_nei)
        if bottom_nei and right_nei in ob_list and rb_nei in neighbor:
            neighbor.remove(rb_nei)
        neighbor = [x for x in neighbor if x not in closed]
        return neighbor

    def find_node_index(self, coordinate, node_list):
        # find node index in the node list via its coordinate
        ind = 0
        for node in node_list:
            if node.coordinate == coordinate:
                target_node = node
                ind = node_list.index(target_node)
                break
        return ind

    def find_path(self, open_list, closed_list, target, bound_obs_and_expand_array):
        # searching for the path, update open and closed list
        # bound_obstacle = obstacle and boundary
        flag = len(open_list)
        for i in range(flag):
            node = open_list[0]
            open_coordinate_list = [node.coordinate for node in open_list]
            closed_coordinate_list = [node.coordinate for node in closed_list]
            temp = self.find_neighbor(node, bound_obs_and_expand_array, closed_coordinate_list)
            for element in temp:
                if element in closed_list:
                    continue
                elif element in open_coordinate_list:
                    # if node in open list, update g value
                    ind = open_coordinate_list.index(element)
                    new_g = self.gcost(node, element)
                    if new_g <= open_list[ind].G:
                        open_list[ind].G = new_g
                        open_list[ind].reset_f()
                        open_list[ind].parent = node
                else:  # new coordinate, create corresponding node
                    ele_node = self.Node(coordinate=element, parent=node,
                                         G=self.gcost(node, element), H=self.hcost(element, target))
                    open_list.append(ele_node)
            open_list.remove(node)
            closed_list.append(node)
            open_list.sort(key=lambda x: x.F)
        return open_list, closed_list

    def node_to_coordinate(self, node_list):
        # convert node list into coordinate list and array
        coordinate_list = [node.coordinate for node in node_list]
        return coordinate_list

    def get_border_line(self, node_closed_ls, obstacle):
        # if no path, find border line which confine goal or robot
        border: list = []
        coordinate_closed_ls = self.node_to_coordinate(node_closed_ls)
        for coordinate in coordinate_closed_ls:
            boundary: list = []
            for x in range(coordinate[0] - 1, coordinate[0] + 2):
                for y in range(coordinate[1] - 1, coordinate[1] + 2):
                    if [x, y] in obstacle:
                        boundary.append([x, y])
            temp = boundary
            border = border + temp
        border_ary = np.array(border)
        return border_ary

    def draw_control(self, org_closed, goal_closed, flag, bound, obstacle, obs_expand_array):
        """
        control the plot process, evaluate if the searching finished
        flag == 0 : draw the searching process and plot path
        flag == 1 or 2 : start or end is blocked, draw the border line
        """
        stop_loop = 0  # stop sign for the searching
        org_closed_ls = self.node_to_coordinate(org_closed)
        org_array = np.array(org_closed_ls)
        goal_closed_ls = self.node_to_coordinate(goal_closed)
        goal_array = np.array(goal_closed_ls)
        path = None
        if show_animation:  # draw the searching process
            # plot the map
            if not goal_array.tolist():  # ensure the close_goal not empty
                # in case of the obstacle number is really large (>4500), the
                # origin is very likely blocked at the first search, and then
                # the program is over and the searching from goal to origin
                # will not start, which remain the closed_list for goal == []
                # in order to plot the map, add the end coordinate to array
                goal_array = np.array([self.end])
            plt.cla()
            plt.gcf().set_size_inches(11, 9, forward=True)
            plt.axis("equal")
            plt.plot(org_array[:, 0], org_array[:, 1], "oy")
            plt.plot(goal_array[:, 0], goal_array[:, 1], "og")
            plt.plot(bound[:, 0], bound[:, 1], "sk")
            plt.plot(obs_expand_array[:, 0], obs_expand_array[:, 1], "oc")
            plt.plot(self.end[0], self.end[1], "*b", label="Goal")
            plt.plot(self.start[0], self.start[1], "^b", label="Origin")
            plt.legend()
            plt.pause(0.0001)
            # draw(org_array, goal_array, start, end, bound)

        if flag == 0:
            cl1 = self.node_to_coordinate(org_closed)
            cl2 = self.node_to_coordinate(goal_closed)
            node_intersect = [node for node in cl1 if node in cl2]
            if node_intersect:  # a path is find
                # get path from intersect to start and end
                path_org: list = []
                path_goal: list = []
                ind = self.find_node_index(node_intersect[0], org_closed)
                node = org_closed[ind]
                while node != org_closed[0]:
                    path_org.append(node.coordinate)
                    node = node.parent
                path_org.append(org_closed[0].coordinate)
                ind = self.find_node_index(node_intersect[0], goal_closed)
                node = goal_closed[ind]
                while node != goal_closed[0]:
                    path_goal.append(node.coordinate)
                    node = node.parent
                path_goal.append(goal_closed[0].coordinate)
                path_org.reverse()
                path = path_org + path_goal
                path = np.array(path)

                # draw the path
                stop_loop = 1
                print("Path is find!")
                if show_animation:
                    plt.plot(path[:, 0], path[:, 1], "-r")
                    plt.title("Arrived", size=20, loc="center")
                    plt.pause(0.01)
                    plt.show()

        elif flag == 1:  # start point blocked first
            stop_loop = 1
            print("There is no path to the goal! Start point is blocked!")
        elif flag == 2:  # end point blocked first
            stop_loop = 1
            print("There is no path to the goal! End point is blocked!")
        if show_animation:  # blocked case, draw the border line
            info = "There is no path to the goal!" \
                   " own_ship&Goal are split by border" \
                   " shown in red \'x\'!"
            if flag == 1:
                border = self.get_border_line(org_closed, obstacle)
                plt.plot(border[:, 0], border[:, 1], "xr")
                plt.title(info, size=14, loc="center")
                plt.pause(0.01)
                plt.show()
            elif flag == 2:
                border = self.get_border_line(goal_closed, obstacle)
                plt.plot(border[:, 0], border[:, 1], "xr")
                plt.title(info, size=14, loc="center")
                plt.pause(0.01)
                plt.show()
        return stop_loop, path

    def searching_control(self, bound_obs, obstacle, obs_expand_array, bound_obs_and_expand_array):
        """manage the searching process, start searching from two side"""
        if self.bottom_vertex[0] < self.start[0] < self.top_vertex[0] and \
                self.bottom_vertex[0] < self.end[0] < self.top_vertex[0] \
                and self.bottom_vertex[1] < self.start[1] < self.top_vertex[1] \
                and self.bottom_vertex[1] < self.end[1] < self.top_vertex[1]:

            # initial origin node and end node
            origin = self.Node(coordinate=self.start, H=self.hcost(self.start, self.end))
            goal = self.Node(coordinate=self.end, H=self.hcost(self.end, self.start))
            # list for searching from origin to goal
            origin_open: list = [origin]
            origin_close: list = []
            # list for searching from goal to origin
            goal_open = [goal]
            goal_close: list = []
            # initial target
            target_goal = self.end
            # flag = 0 (not blocked) 1 (start point blocked) 2 (end point blocked)
            flag = 0  # init flag
            path = None
            while True:
                # searching from start to end
                origin_open, origin_close = \
                    self.find_path(origin_open, origin_close, target_goal, bound_obs_and_expand_array)
                if not origin_open:  # no path condition
                    flag = 1  # origin node is blocked
                    self.draw_control(origin_close, goal_close, flag, bound_obs, obstacle, obs_expand_array)
                    break
                # update target for searching from end to start
                target_origin = min(origin_open, key=lambda x: x.F).coordinate

                # searching from end to start
                goal_open, goal_close = \
                    self.find_path(goal_open, goal_close, target_origin, bound_obs_and_expand_array)
                if not goal_open:  # no path condition
                    flag = 2  # goal is blocked
                    self.draw_control(origin_close, goal_close, flag, bound_obs, obstacle, obs_expand_array)
                    break
                # update target for searching from start to end
                target_goal = min(goal_open, key=lambda x: x.F).coordinate

                # continue searching, draw the process
                stop_sign, path = self.draw_control(origin_close, goal_close, flag, bound_obs, obstacle,
                                                    obs_expand_array)
                if stop_sign:
                    break
            path_plan = Path()
            path_plan.add_point(path)
            return path_plan

        else:
            print("start or end are out of bound")


# --------------------------------------------------------------------------- #
# Test Astar
# --------------------------------------------------------------------------- #
def test_path_plan_Astar():
    print(__file__ + " start!")

    top_vertex = [60, 60]  # top right vertex of boundary
    bottom_vertex = [0, 0]  # bottom left vertex of boundary

    obstacles = np.vstack((np.random.randint(bottom_vertex[0] + 1, top_vertex[0], 50).tolist(),
                           np.random.randint(bottom_vertex[1] + 1, top_vertex[1], 50).tolist()))

    start = [np.random.randint(bottom_vertex[0] + 1, top_vertex[0]),
             np.random.randint(bottom_vertex[1] + 1, top_vertex[1])]

    end = [np.random.randint(bottom_vertex[0] + 1, top_vertex[0]),
           np.random.randint(bottom_vertex[1] + 1, top_vertex[1])]

    test_path_plan = AStar(start, end, top_vertex, bottom_vertex, obstacles)
    bound_obs, obstacle, obstacle_expand, bound_obs_and_expand_array = test_path_plan.boundary_and_obstacles()

    path = test_path_plan.searching_control(bound_obs, obstacle, obstacle_expand, bound_obs_and_expand_array)
    if not show_animation:
        print(path)
# --------------------------------------------------------------------------- #
# Main function for tests
# --------------------------------------------------------------------------- #
if __name__ == '__main__':
    test_path_plan_Astar()
    # test_path_plan_APF()

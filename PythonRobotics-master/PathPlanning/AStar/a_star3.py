import heapq
import matplotlib.pyplot as plt


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.g = 0
        self.h = 0
        self.f = 0
        self.parent = None

    def __lt__(self, other):
        return self.f < other.f

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y


def astar(start, end, grid):
    open_list = []
    closed_list = []
    heapq.heappush(open_list, start)

    while open_list:
        current_node = heapq.heappop(open_list)
        closed_list.append(current_node)

        if current_node == end:
            path = []
            while current_node:
                path.append((current_node.x, current_node.y))
                current_node = current_node.parent
            return path[::-1]

        neighbors = [(current_node.x + 1, current_node.y),
                     (current_node.x - 1, current_node.y),
                     (current_node.x, current_node.y + 1),
                     (current_node.x, current_node.y - 1)]

        for neighbor in neighbors:
            x, y = neighbor

            if x < 0 or y < 0 or x >= len(grid) or y >= len(grid[0]):
                continue

            if grid[x][y] == 1:
                continue

            neighbor_node = Node(x, y)
            neighbor_node.g = current_node.g + 1
            neighbor_node.h = ((x - end.x) ** 2 + (y - end.y) ** 2) ** 0.5
            neighbor_node.f = neighbor_node.g + neighbor_node.h

            if neighbor_node in closed_list:
                continue

            if neighbor_node in open_list:
                existing_node = open_list[open_list.index(neighbor_node)]
                if existing_node.g > neighbor_node.g:
                    existing_node.g = neighbor_node.g
                    existing_node.parent = current_node
                    existing_node.f = existing_node.g + existing_node.h
            else:
                neighbor_node.parent = current_node
                heapq.heappush(open_list, neighbor_node)

    return None

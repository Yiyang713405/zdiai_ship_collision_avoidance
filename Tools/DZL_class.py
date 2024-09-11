# -*- coding: utf-8 -*-

# --------------------------------------------------------------------------- #
# Documentation
# --------------------------------------------------------------------------- #
__author__ = ''
__date__ = ''
__email__ = ''
__copyright__ = ''

# --------------------------------------------------------------------------- #
# Import
# --------------------------------------------------------------------------- #
import numpy as np
from matplotlib import pyplot as plt

# --------------------------------------------------------------------------- #
# Path
# --------------------------------------------------------------------------- #
class Path(object):

    def __init__(self):
        self.count = 0  # Path path_point count
        self.coord = None  # Path coordinates in ECEF coordinate system
        self.num = 0  # Number of path points
        self.arr = 0

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
        return "\n".join(["%s:\t%s" % item for item in self.__dict__.items()])

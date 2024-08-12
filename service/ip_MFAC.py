# -*- coding: utf-8 -*-
"""
ip_control implements methods for controllers.
"""
# --------------------------------------------------------------------------- #
# Import
# --------------------------------------------------------------------------- #
import time
import numpy as np


# --------------------------------------------------------------------------- #
# Documentation
# --------------------------------------------------------------------------- #
__author__ = "XCQ"
__date__ = "2018/8/5"
__email__ = "jialunliu@outlook.com"
__copyright__ = "Copyright (c) 2018 Jialun Liu. All rights reserved."


# --------------------------------------------------------------------------- #
# Definition
# --------------------------------------------------------------------------- #
class MFAC(object):
    """
    A model-free adaptive controller.
    """

    def __init__(self, sample_time, rho, lamda, eta, mu, phi1, epsilon, output_min=float("-inf"),
                 output_max=float("inf")):
        if rho is None:
            raise ValueError("rho must be specified.")
        if lamda is None:
            raise ValueError("lamda must be specified.")
        if eta is None:
            raise ValueError("eta must be specified.")
        if mu is None:
            raise ValueError("mu must be specified.")
        if phi1 is None:
            raise ValueError("phi1 must be specified.")
        if epsilon is None:
            raise ValueError("epsilon must be specified.")
        if sample_time <= 0:
            raise ValueError("sample_time must be greater than 0.")
        if output_min >= output_max:
            raise ValueError("out_min must be less than out_max.")

        self._rho = rho
        self._lamda = lamda
        self._eta = eta
        self._mu = mu
        self._phi1 = phi1
        self._epsilon = epsilon
        self._sample_time = sample_time * 1000
        self._output_val_min_rad = np.deg2rad(output_min)
        self._output_val_max_rad = np.deg2rad(output_max)
        self._last_output_val = 0
        self._last_input_val_rad = 0
        self._last_output_val_rad = 0
        self._last2_output_val_rad = 0
        self._last_get_time = 0
        self._phi = phi1

    def solve(self, target_val, input_val):
        """
        Adjust and hold the given target value.

        Args:
            target_val (float): The target heading angle.
            input_val (float): The actual heading angle.

        Returns:
            A command of rudder angle between `output_min` and `output_max`.
        """
        current_time = time.time() * 1000

        if (current_time - self._last_get_time) < self._sample_time:
            return self._last_output_val

        # Change Deg To Rad
        target_val_rad = np.deg2rad(target_val)
        input_val_rad = np.deg2rad(input_val)

        # Compute all the working error variables
        error = target_val_rad - input_val_rad  # error between target heading angle and actual heading angle
        input_diff = input_val_rad - self._last_input_val_rad  # difference of actual heading angle
        last_output_diff_rad = self._last_output_val_rad - self._last2_output_val_rad  # difference of rudder angle

        self._phi += (self._eta * last_output_diff_rad / (self._mu + (abs(last_output_diff_rad))**2)) * (
                input_diff - self._phi * last_output_diff_rad)   # 偏导
        if abs(self._phi) <= self._epsilon or abs(last_output_diff_rad) <= self._epsilon:
            self._phi = self._phi1

        # Compute MFAC Rudder Output
        self._last2_output_val_rad = self._last_output_val_rad
        self._last_output_val_rad += (self._rho * self._phi / (self._lamda + (abs(self._phi))**2)) * error
        self._last_output_val_rad = min(self._last_output_val_rad, self._output_val_max_rad)  # 输出信号极限限制
        self._last_output_val_rad = max(self._last_output_val_rad, self._output_val_min_rad)  # 输出信号极限限制
        self._last_output_val = np.rad2deg(self._last_output_val_rad)

        # Remember some variables for next time
        self._last_input_val_rad = input_val_rad
        self._last_get_time = current_time
        return self._last_output_val

# class MFAC_u(object):
#     """
#     A model-free adaptive controller.
#     """
#
#     def __init__(self, rho, lamda, eta, mu, phi1, epsilon, output_min=float("-inf"),
#                  output_max=float("inf")):
#         if rho is None:
#             raise ValueError("rho must be specified.")
#         if lamda is None:
#             raise ValueError("lamda must be specified.")
#         if eta is None:
#             raise ValueError("eta must be specified.")
#         if mu is None:
#             raise ValueError("mu must be specified.")
#         if phi1 is None:
#             raise ValueError("phi1 must be specified.")
#         if epsilon is None:
#             raise ValueError("epsilon must be specified.")
#         # if sample_time <= 0:
#         #     raise ValueError("sample_time must be greater than 0.")
#         if output_min >= output_max:
#             raise ValueError("out_min must be less than out_max.")
#
#         self._rho = rho
#         self._lamda = lamda
#         self._eta = eta
#         self._mu = mu
#         self._phi1 = phi1
#         self._epsilon = epsilon
#         # self._sample_time = sample_time * 1000
#         self._output_val_min_rad = np.deg2rad(output_min)
#         self._output_val_max_rad = np.deg2rad(output_max)
#         self._last_output_val = 0
#         self._last_input_val_rad = 0
#         self._last_output_val_rad = 0
#         self._last2_output_val_rad = 0
#         self._last_get_time = 0
#         self._phi = phi1
#
#     def solve(self, target_val, input_val):
#         """
#         Adjust and hold the given target value.
#
#         Args:
#             target_val (float): The target heading angle.
#             input_val (float): The actual heading angle.
#
#         Returns:
#             A command of rudder angle between `output_min` and `output_max`.
#         """
#         # current_time = time.time() * 1000
#         #
#         # if (current_time - self._last_get_time) < self._sample_time:
#         #     return self._last_output_val
#
#         # Change Deg To Rad
#         target_val_rad = np.deg2rad(target_val)
#         input_val_rad = np.deg2rad(input_val)
#
#         # Compute all the working error variables
#         error = target_val_rad - input_val_rad  # error between target heading angle and actual heading angle
#         input_diff = input_val_rad - self._last_input_val_rad  # difference of actual heading angle
#         last_output_diff_rad = self._last_output_val_rad - self._last2_output_val_rad  # difference of rudder angle
#
#         self._phi += (self._eta * last_output_diff_rad / (self._mu + (abs(last_output_diff_rad))**2)) * (
#                 input_diff - self._phi * last_output_diff_rad)   # 偏导
#         if abs(self._phi) <= self._epsilon or abs(last_output_diff_rad) <= self._epsilon:
#             self._phi = self._phi1
#
#         # Compute MFAC Rudder Output
#         self._last2_output_val_rad = self._last_output_val_rad
#         self._last_output_val_rad += (self._rho * self._phi / (self._lamda + (abs(self._phi))**2)) * error
#         self._last_output_val_rad = min(self._last_output_val_rad, self._output_val_max_rad)  # 输出信号极限限制
#         self._last_output_val_rad = max(self._last_output_val_rad, self._output_val_min_rad)  # 输出信号极限限制
#         self._last_output_val = np.rad2deg(self._last_output_val_rad)
#
#         # Remember some variables for next time
#         self._last_input_val_rad = input_val_rad
#         # self._last_get_time = current_time
#         return self._last_output_val

class MFAC_u(object):
    """
    A model-free adaptive controller.
    """

    def __init__(self, rho, lamda, eta, mu, phi1, epsilon, output_min=float("-inf"),
                 output_max=float("inf")):
        if rho is None:
            raise ValueError("rho must be specified.")
        if lamda is None:
            raise ValueError("lamda must be specified.")
        if eta is None:
            raise ValueError("eta must be specified.")
        if mu is None:
            raise ValueError("mu must be specified.")
        if phi1 is None:
            raise ValueError("phi1 must be specified.")
        if epsilon is None:
            raise ValueError("epsilon must be specified.")
        # if sample_time <= 0:
        #     raise ValueError("sample_time must be greater than 0.")
        if output_min >= output_max:
            raise ValueError("out_min must be less than out_max.")

        self._rho = rho
        self._lamda = lamda
        self._eta = eta
        self._mu = mu
        self._phi1 = phi1
        self._epsilon = epsilon
        # self._sample_time = sample_time * 1000
        self._output_val_min = output_min
        self._output_val_max= output_max
        self._last_output_val = 0
        self._last_input_val = 0
        self._last_output_val = 0
        self._last2_output_val = 0
        self._last_get_time = 0
        self._phi = phi1

    def solve(self, target_val, input_val):
        """
        Adjust and hold the given target value.

        Args:
            target_val (float): The target heading angle.
            input_val (float): The actual heading angle.

        Returns:
            A command of rudder angle between `output_min` and `output_max`.
        """
        # current_time = time.time() * 1000
        #
        # if (current_time - self._last_get_time) < self._sample_time:
        #     return self._last_output_val

        # Compute all the working error variables
        error = target_val - input_val  # error between target heading angle and actual heading angle
        input_diff = input_val - self._last_input_val  # difference of actual heading angle
        last_output_diff = self._last_output_val - self._last2_output_val  # difference of rudder angle

        self._phi += (self._eta * last_output_diff / (self._mu + (abs(last_output_diff)) ** 2)) * (
                input_diff - self._phi * last_output_diff)  # 偏导
        if abs(self._phi) <= self._epsilon or abs(last_output_diff) <= self._epsilon:
            self._phi = self._phi1

        # Compute MFAC Rudder Output
        self._last2_output_val = self._last_output_val
        self._last_output_val += (self._rho * self._phi / (self._lamda + (abs(self._phi)) ** 2)) * error
        self._last_output_val = min(self._last_output_val, self._output_val_max)  # 输出信号极限限制
        self._last_output_val = max(self._last_output_val, self._output_val_min)  # 输出信号极限限制

        # Remember some variables for next time
        self._last_input_val = input_val
        # self._last_get_time = current_time
        return self._last_output_val


class RO_MFAC(object):
    """
    An active disturbance rejection model-free adaptive controller.
    """

    def __init__(self, rho, lamda, eta, mu, phi1, epsilon, K1, output_min=float("-inf"),
                 output_max=float("inf")):
        if rho is None:
            raise ValueError("rho must be specified.")
        if lamda is None:
            raise ValueError("lamda must be specified.")
        if eta is None:
            raise ValueError("eta must be specified.")
        if mu is None:
            raise ValueError("mu must be specified.")
        if phi1 is None:
            raise ValueError("phi1 must be specified.")
        if epsilon is None:
            raise ValueError("epsilon must be specified.")
        if output_min >= output_max:
            raise ValueError("out_min must be less than out_max.")

        self._rho = rho
        self._lamda = lamda
        self._eta = eta
        self._mu = mu
        self._phi1 = phi1
        self._epsilon = epsilon
        self._K1 = K1
        self._output_val_min = output_min
        self._output_val_max = output_max
        self._last_output_val = 0
        self._last_input_val = 0
        self._last_output_val = 0
        self._last2_output_val = 0
        self._last_input_r = 0
        self._phi = phi1

    def solve(self, target_val, input_val, input_r):
        """
        Adjust and hold the given target value.

        Args:
            target_val (float): The target heading angle.
            input_val (float): The actual heading angle.
            input_r (float): The actual angular velocity.

        Returns:
            A command of rudder angle between `output_min` and `output_max`.
        """

        input_Y = input_val+ self._K1 * input_r   # y = psi + k*r

        # Compute all the working error and change of the variables
        error_psi = target_val - input_val              # error between target psi and actual psi
        error_r = 0 - input_r                                # error of r
        error_Y = target_val - input_Y                   # error of y
        input_diff = input_val - self._last_input_val    # change of actual psi
        r_diff = input_r - self._last_input_r            # change of r
        Y_diff = input_diff + self._K1 * r_diff                  # change of y

        last_output_diff = self._last_output_val - self._last2_output_val  # last change of rudder angle

        # Calculate the pseudo partial derivative
        self._phi += (self._eta * last_output_diff / (self._mu + (abs(last_output_diff))**2)) * (
                Y_diff - self._phi * last_output_diff)
        if abs(self._phi) <= self._epsilon or abs(last_output_diff) <= self._epsilon:
            self._phi = self._phi1

        # self._phi1 += (self._eta1 * last_output_diff_rad / (self._mu1 + (abs(last_output_diff_rad)) ** 2)) * (
        #         input_diff - self._phi1 * last_output_diff_rad)
        # self._phi2 += (self._eta2 * last_output_diff_rad / (self._mu2 + (abs(last_output_diff_rad)) ** 2)) * (
        #         r_diff - self._phi2 * last_output_diff_rad)
        # if abs(self._phi1) <= self._epsilon1 or abs(last_output_diff_rad) <= self._epsilon1:
        #     self._phi1 = self._phi1a
        # if abs(self._phi2) <= self._epsilon2 or abs(last_output_diff_rad) <= self._epsilon2:
        #     self._phi2 = self._phi1b

        # Compute MFAC Rudder Output
        self._last2_output_val = self._last_output_val
        self._last_output_val += (self._rho * self._phi / (self._lamda + (abs(self._phi))**2)) * error_Y

        self._last_output_val = min(self._last_output_val, self._output_val_max)
        self._last_output_val = max(self._last_output_val, self._output_val_min)

        # Remember some variables for next time
        self._last_input_val = input_val
        self._last_input_r = input_r
        return np.rad2deg(self._last_output_val)




# if __name__ == "__main__":
#     test_pid = PID(0.1, 1, 2, 3)
#     print(test_pid.get_pid_param())
#     test_pid.update_pid_param(4, 5, 6)
#     print(test_pid.get_pid_param())

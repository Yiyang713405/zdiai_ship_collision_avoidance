# -*- coding: utf-8 -*-
"""
ip_control implements methods for controllers.
"""
# --------------------------------------------------------------------------- #
# Import
# --------------------------------------------------------------------------- #
import time

# --------------------------------------------------------------------------- #
# Documentation
# --------------------------------------------------------------------------- #
__author__ = "Jialun Liu"
__date__ = "2018/8/5"
__email__ = "jialunliu@outlook.com"
__copyright__ = "Copyright (c) 2018 Jialun Liu. All rights reserved."


# --------------------------------------------------------------------------- #
# Definition
# --------------------------------------------------------------------------- #
class Turning(object):
    # Controller for turning tests
    def __init__(self):
        self.turning_delta_deg = None
        self.cmd_delta_deg = None

    def set_param(self, turning_delta_deg):
        self.turning_delta_deg = turning_delta_deg

    def get_param(self):
        return self.turning_delta_deg

    def solve(self):
        self.cmd_delta_deg = self.turning_delta_deg
        return self.cmd_delta_deg


class Zigzag(object):
    # Controller for zigzag tests
    def __init__(self):
        self.zigzag_psi_deg = None
        self.zigzag_delta_deg = None
        self.cmd_delta_deg = None

    def set_param(self, zigzag_delta_deg, zigzag_psi_deg):
        self.zigzag_psi_deg = zigzag_psi_deg
        self.zigzag_delta_deg = zigzag_delta_deg

    def get_param(self):
        return self.zigzag_delta_deg, self.zigzag_psi_deg

    def solve(self, psi_deg_curr, psi_deg_prev):
        if psi_deg_curr > 180:
            psi_deg = psi_deg_curr - 360
        else:
            psi_deg = psi_deg_curr
        if psi_deg >= self.zigzag_psi_deg:
            self.cmd_delta_deg = -self.zigzag_delta_deg
        elif psi_deg < -self.zigzag_psi_deg:
            self.cmd_delta_deg = self.zigzag_delta_deg
        elif (abs(psi_deg) <= self.zigzag_psi_deg) & ((psi_deg_curr - psi_deg_prev) < 0.0):
            self.cmd_delta_deg = -self.zigzag_delta_deg
        elif (abs(psi_deg) <= self.zigzag_psi_deg) & ((psi_deg_curr - psi_deg_prev) >= 0.0):
            self.cmd_delta_deg = self.zigzag_delta_deg
        return self.cmd_delta_deg


class PID(object):
    """
    A proportional-integral-derivative controller based on Arduino PID Library.

    https://github.com/br3ttb/Arduino-PID-Library
    http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
    http://brettbeauregard.com/blog/2011/04/improving-the-beginner%e2%80%99s-pid-sample-time/
    http://brettbeauregard.com/blog/2011/04/improving-the-beginner%e2%80%99s-pid-derivative-kick/
    http://brettbeauregard.com/blog/2011/04/improving-the-beginner%e2%80%99s-pid-tuning-changes/
    http://brettbeauregard.com/blog/2011/04/improving-the-beginner%e2%80%99s-pid-reset-windup/
    http://brettbeauregard.com/blog/2011/04/improving-the-beginner%e2%80%99s-pid-initialization/
    http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-direction/

    Args:
        sample_time (float): The interval between solve() calls.
        kp (float): Proportional coefficient.
        ki (float): Integral coefficient.
        kd (float): Derivative coefficient.
        output_min (float): Lower output limit.
        output_max (float): Upper output limit.
    """

    def __init__(self, sample_time, kp, ki, kd, output_min=float("-inf"), output_max=float("inf")):
        if kp is None:
            raise ValueError("kp must be specified.")
        if ki is None:
            raise ValueError("ki must be specified.")
        if kd is None:
            raise ValueError("kd must be specified.")
        if sample_time <= 0:
            raise ValueError("sample_time must be greater than 0.")
        if output_min >= output_max:
            raise ValueError("out_min must be less than out_max.")

        self._kp = kp
        self._ki = ki * sample_time
        self._kd = kd / sample_time
        self._sample_time = sample_time * 1000
        self._output_val_min = output_min
        self._output_val_max = output_max
        self._integral = 0
        self._last_input_val = 0
        self._last_output_val = 0
        self._last_get_time = 0

    def set_pid_param(self, kp, ki, kd):
        self._kp = kp
        self._ki = ki * self._sample_time / 1000
        self._kd = kd / self._sample_time * 1000

    def get_pid_param(self):
        return self._kp, self._ki / self._sample_time * 1000, self._kd * self._sample_time / 1000

    def solve(self, target_val, input_val):
        """Adjust and hold the given target value.

        Args:
            target_val (float): The target value.
            input_val (float): The input value.

        Returns:
            A value between `output_min` and `output_max`.
        """
        current_time = time.time() * 1000

        if (current_time - self._last_get_time) < self._sample_time:
            return self._last_output_val

        # Compute all the working error variables
        error = target_val - input_val
        input_diff = input_val - self._last_input_val

        # In order to prevent windup, only integrate if the process is not saturated
        if self._output_val_max > self._last_output_val > self._output_val_min:
            self._integral += self._ki * error
            self._integral = min(self._integral, self._output_val_max)
            self._integral = max(self._integral, self._output_val_min)

        p = self._kp * error
        i = self._integral
        d = -(self._kd * input_diff)

        # Compute PID Output
        self._last_output_val = p + i + d
        self._last_output_val = min(self._last_output_val, self._output_val_max)
        self._last_output_val = max(self._last_output_val, self._output_val_min)

        # Remember some variables for next time
        self._last_input_val = input_val
        self._last_get_time = current_time
        return self._last_output_val


if __name__ == "__main__": # 这句话后面的作用是调试控制器
    test_pid = PID(0.1, 1, 2, 3)
    print(test_pid.get_pid_param())
    test_pid.update_pid_param(4, 5, 6)
    print(test_pid.get_pid_param())

"""

Extended kalman filter (EKF) localization sample

author: Atsushi Sakai (@Atsushi_twi)

"""
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

import math
import matplotlib.pyplot as plt
import numpy as np

from utils.angle import rot_mat_2d
from Localization.unscented_kalman_filter import unscented_kalman_filter as ukfest

# Covariance for EKF simulation
Q = np.diag([
    0.1,  # variance of location on x-axis
    0.1,  # variance of location on y-axis
    np.deg2rad(1.0),  # variance of yaw angle
    1.0  # variance of velocity
]) ** 2  # predict state covariance
R = np.diag([1.0, 1.0]) ** 2  # Observation x,y position covariance

#  Simulation parameter
INPUT_NOISE = np.diag([1.0, np.deg2rad(30.0)]) ** 2
# GPS_NOISE = np.diag([0.5, 0.5]) ** 2
GPS_NOISE = np.diag([1, 1]) ** 2

DT = 0.1  # time tick [s]
SIM_TIME = 100.0  # simulation time [s]

show_animation = True


def calc_input():
    v = 5.0  # [m/s]
    yawrate = 0.1  # [rad/s]
    u = np.array([[v], [yawrate]])
    return u


def observation(xTrue, xd, u):
    xTrue = motion_model(xTrue, u)

    # add noise to gps x-y
    z = observation_model(xTrue) + GPS_NOISE @ np.random.randn(2, 1)

    # add noise to input
    ud = u + INPUT_NOISE @ np.random.randn(2, 1)

    xd = motion_model(xd, ud)

    return xTrue, z, xd, ud


def motion_model(x, u):
    F = np.array([[1.0, 0, 0, 0],
                  [0, 1.0, 0, 0],
                  [0, 0, 1.0, 0],
                  [0, 0, 0, 0]])

    # B = np.array([[DT * math.cos(x[2, 0]), 0],
    #               [DT * math.sin(x[2, 0]), 0],
    #               [0.0, DT],
    #               [1.0, 0.0]])

    B = np.array([[DT * math.cos(x[2, 0]), 0],
                  [DT * math.sin(x[2, 0]), 0],
                  [0.0, -0.05],
                  [1.0, 0.0]])

    x = F @ x + B @ u

    return x


def observation_model(x):
    H = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

    z = H @ x

    return z


def jacob_f(x, u):
    """
    Jacobian of Motion Model

    motion model
    x_{t+1} = x_t+v*dt*cos(yaw)
    y_{t+1} = y_t+v*dt*sin(yaw)
    yaw_{t+1} = yaw_t+omega*dt
    v_{t+1} = v{t}
    so
    dx/dyaw = -v*dt*sin(yaw)
    dx/dv = dt*cos(yaw)
    dy/dyaw = v*dt*cos(yaw)
    dy/dv = dt*sin(yaw)
    """
    yaw = x[2, 0]
    v = u[0, 0]
    jF = np.array([
        [1.0, 0.0, -DT * v * math.sin(yaw), DT * math.cos(yaw)],
        [0.0, 1.0, DT * v * math.cos(yaw), DT * math.sin(yaw)],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]])

    return jF


def jacob_h():
    # Jacobian of Observation Model
    jH = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

    return jH


def ekf_estimation(xEst, PEst, z, u):
    #  Predict
    xPred = motion_model(xEst, u)
    jF = jacob_f(xEst, u)
    PPred = jF @ PEst @ jF.T + Q

    #  Update
    jH = jacob_h()
    zPred = observation_model(xPred)
    y = z - zPred
    S = jH @ PPred @ jH.T + R
    K = PPred @ jH.T @ np.linalg.inv(S)
    xEst = xPred + K @ y
    PEst = (np.eye(len(xEst)) - K @ jH) @ PPred
    return xEst, PEst


def plot_covariance_ellipse(xEst, PEst):  # pragma: no cover
    Pxy = PEst[0:2, 0:2]
    eigval, eigvec = np.linalg.eig(Pxy)

    if eigval[0] >= eigval[1]:
        bigind = 0
        smallind = 1
    else:
        bigind = 1
        smallind = 0

    t = np.arange(0, 2 * math.pi + 0.1, 0.1)
    a = math.sqrt(eigval[bigind])
    b = math.sqrt(eigval[smallind])
    x = [a * math.cos(it) for it in t]
    y = [b * math.sin(it) for it in t]
    angle = math.atan2(eigvec[1, bigind], eigvec[0, bigind])
    fx = rot_mat_2d(angle) @ (np.array([x, y]))
    px = np.array(fx[0, :] + xEst[0, 0]).flatten()
    py = np.array(fx[1, :] + xEst[1, 0]).flatten()
    plt.plot(px, py, "--r")


def main():
    print(__file__ + " start!!")

    time = 0.0

    # State Vector [x y yaw v]'
    # xEst = np.zeros((4, 1))
    xEst = np.array([[0], [500], [math.pi/2], [5]])
    # xTrue = np.zeros((4, 1))
    xTrue = np.array([[0], [500], [math.pi/2], [5]])
    PEst = np.eye(4)
    # xDR = np.zeros((4, 1))  # Dead reckoning
    xDR = np.array([[0], [500], [math.pi/2], [5]])

    # history
    hxEst = xEst
    hxEst_u = xEst
    hxTrue = xTrue
    hxDR = xTrue
    # hz = np.zeros((2, 1))
    hz = np.array([[0], [500]])
    wm, wc, gamma = ukfest.setup_ukf(4)

    while SIM_TIME >= time:
        time += DT
        u = calc_input()

        xTrue, z, xDR, ud = observation(xTrue, xDR, u)
        xEst, PEst = ekf_estimation(xEst, PEst, z, ud)

        xEst_u, PEst_u = ukfest.ukf_estimation(xEst, PEst, z, ud, wm, wc, gamma)

        # store data history
        hxEst = np.hstack((hxEst, xEst))
        hxEst_u = np.hstack((hxEst_u, xEst_u))
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))
        hz = np.hstack((hz, z))

        if show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(hz[0, :], hz[1, :], ".g")
            plt.plot(hxTrue[0, :].flatten(),
                     hxTrue[1, :].flatten(), "-b")
            plt.plot(hxDR[0, :].flatten(),
                     hxDR[1, :].flatten(), "-k")
            plt.plot(hxEst[0, :].flatten(),
                     hxEst[1, :].flatten(), "-r")
            plt.plot(hxEst_u[0, :].flatten(),
                     hxEst_u[1, :].flatten(), "-m")
            plot_covariance_ellipse(xEst, PEst)
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)


def main2():
    print(__file__ + " start!!")

    time = 0.0

    # State Vector [x y yaw v]'
    xEst1 = np.array([[50], [550], [math.pi], [5]])
    xEst2 = np.array([[0], [500], [-math.pi/2], [5]])
    xTrue1 = np.array([[50], [550], [math.pi], [5]])
    xTrue2 = np.array([[0], [500], [-math.pi/2], [5]])
    PEst1 = np.eye(4)
    PEst2 = np.eye(4)

    xDR1 = np.array([[50], [550], [math.pi], [5]])
    xDR2 = np.array([[0], [500], [-math.pi/2], [5]])

    # history
    hxEst1 = xEst1
    hxEst2 = xEst2
    hxEst_u1 = xEst1
    hxEst_u2 = xEst2
    hxTrue1 = xTrue1
    hxTrue2 = xTrue2
    hxDR1 = xTrue1
    hxDR2 = xTrue2
    hz1 = np.array([[50], [550]])
    hz2 = np.array([[0], [500]])
    wm, wc, gamma = ukfest.setup_ukf(4)

    while SIM_TIME >= time:
        time += DT
        u = calc_input()

        xTrue1, z1, xDR1, ud1 = observation(xTrue1, xDR1, u)
        xTrue2, z2, xDR2, ud2 = observation(xTrue2, xDR2, u)

        xEst1, PEst1 = ekf_estimation(xEst1, PEst1, z1, ud1)
        xEst2, PEst2 = ekf_estimation(xEst2, PEst2, z2, ud2)

        xEst_u1, PEst_u1 = ukfest.ukf_estimation(xEst1, PEst1, z1, ud1, wm, wc, gamma)
        xEst_u2, PEst_u2 = ukfest.ukf_estimation(xEst2, PEst2, z2, ud2, wm, wc, gamma)

        # store data history
        hxEst1 = np.hstack((hxEst1, xEst1))
        hxEst2 = np.hstack((hxEst2, xEst2))
        hxEst_u1 = np.hstack((hxEst_u1, xEst_u1))
        hxEst_u2 = np.hstack((hxEst_u2, xEst_u2))
        hxDR1 = np.hstack((hxDR1, xDR1))
        hxDR2 = np.hstack((hxDR2, xDR2))
        hxTrue1 = np.hstack((hxTrue1, xTrue1))
        hxTrue2 = np.hstack((hxTrue2, xTrue2))
        hz1 = np.hstack((hz1, z1))
        hz2 = np.hstack((hz2, z2))

        if show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(hz1[0, :], hz1[1, :], ".g")
            plt.plot(hz2[0, :], hz2[1, :], ".g")
            plt.plot(hxTrue1[0, :].flatten(), hxTrue1[1, :].flatten(), "-b")
            plt.plot(hxTrue2[0, :].flatten(), hxTrue2[1, :].flatten(), "-b")
            plt.plot(hxDR1[0, :].flatten(), hxDR1[1, :].flatten(), "-k")
            plt.plot(hxDR2[0, :].flatten(), hxDR2[1, :].flatten(), "-k")
            plt.plot(hxEst1[0, :].flatten(), hxEst1[1, :].flatten(), "-r")
            plt.plot(hxEst2[0, :].flatten(), hxEst2[1, :].flatten(), "-r")
            plt.plot(hxEst_u1[0, :].flatten(), hxEst_u1[1, :].flatten(), "-m")
            plt.plot(hxEst_u2[0, :].flatten(), hxEst_u2[1, :].flatten(), "-m")
            plot_covariance_ellipse(xEst1, PEst1)
            plot_covariance_ellipse(xEst2, PEst2)
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)


if __name__ == '__main__':
    main()
    # main2()

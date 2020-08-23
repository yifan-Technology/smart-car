#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Aug 22 17:42:26 2020

@author: yf
"""

#####################################################################################################
import math
from enum import Enum
import matplotlib.pyplot as plt
import time
import numpy as np

show_animation = True
m = 5
g = 9.80665
mu = 0.01
x0 = 0.01

a = 0.661 / 2
b = 0.661 / 2
c = 0.504 / 2
h = 0.4
rho = 2 * 1000 #density

# I = 2 / 3 * rho * h * (c * (b ** 3 + a ** 3) + (c ** 3 * (a + b)))
r = 0.095
# N14 = b / (2 * (a + b)) * m * g
# N23 = a / (2 * (a + b)) * m * g
# ks = 1
#
# R_a = 1
# L_a = 0.01
# J = 1 / 12 * 0.4 * r ** 2
# K1 = 1 / 340
# K2 = 0.0202


def koordianten_transformation(wheel_speed, theta):
    omega_l = wheel_speed[0]
    omega_r = wheel_speed[1]

    vx = r * (omega_l + omega_r) / 2
    omega = r * (- omega_l + omega_r) / (2 * c)
    vy = -x0 * omega

    dX = np.cos(theta) * vx - np.sin(theta) * vy
    dY = np.sin(theta) * vx + np.cos(theta) * vy
    inertial_velo = np.array([dX, dY])
    return inertial_velo, vx, omega


def dwa_control(x, config, goal, ob):
    """
    Dynamic Window Approach control
    """

    dw = calc_dynamic_window(x, config)

    u, trajectory, all_trajctory = calc_control_and_trajectory(x, dw, config, goal, ob)
    Vtrans = np.array([[1, -c], [1, c]]) / r
    wheel_speed = np.dot(Vtrans, u)

    return wheel_speed, trajectory, all_trajctory


class RobotType(Enum):
    circle = 0
    rectangle = 1


class Config:
    """
    simulation parameter class
    """

    def __init__(self):
        # robot parameter
        self.max_speed = 3.0  # [m/s]
        self.min_speed = -2.5  # [m/s]
        self.max_yaw_rate = 180.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 5.0  # [m/ss]
        self.max_delta_yaw_rate = 180.0 * math.pi / 180.0  # [rad/ss]
        self.v_resolution = 0.2  # [m/s]
        self.yaw_rate_resolution = 5. * math.pi / 180.0  # [rad/s]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.predict_time = 1.8  # [s]  less and more flexible
        self.to_goal_cost_gain = 0.19025
        self.speed_cost_gain = 0.394
        self.obstacle_cost_gain = 1.29002
        self.robot_type = RobotType.circle

        # if robot_type == RobotType.circle
        # Also used to check if goal is reached in both types
        self.robot_radius = 0.6  # [m] for collision check

        # if robot_type == RobotType.rectangle
        self.robot_width = 2*c + 0.2  # [m] for collision check
        self.robot_length = a + b + 0.2  # [m] for collision check

    @property
    def robot_type(self):
        return self._robot_type

    @robot_type.setter
    def robot_type(self, value):
        if not isinstance(value, RobotType):
            raise TypeError("robot_type must be an instance of RobotType")
        self._robot_type = value


def motion(x, u, dt):
    """
    motion model
    """
    x[2] += x[4] * dt
    velo, vx, omega = koordianten_transformation(u, x[2])

    x[0] += velo[0] * dt
    x[1] += velo[1] * dt
    x[3] = vx
    x[4] = omega

    return x


def calc_dynamic_window(x, config):
    """
    calculation dynamic window based on current state x
    """

    # Dynamic window from robot specification
    Vs = [config.min_speed, config.max_speed,
          -config.max_yaw_rate, config.max_yaw_rate]

    # Dynamic window from motion model
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_delta_yaw_rate * config.dt,
          x[4] + config.max_delta_yaw_rate * config.dt]

    #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    return dw


def predict_trajectory(x_init, v_soll, omega_soll, config):
    """
    predict trajectory with an input
    """
    speed_soll = np.array([v_soll, omega_soll])
    x = np.array(x_init)
    trajectory = np.copy(x)
    time = 0<= config.robot_radius
    Vtrans = np.array([[1, -c], [1, c]]) / r
    wheel_speed = np.dot(Vtrans, speed_soll)
    while time <= config.predict_time:  # now is 2s
        x = motion(x, wheel_speed, config.dt)
        trajectory = np.vstack((trajectory, x))
        time += config.dt

    return trajectory


def calc_control_and_trajectory(x, dw, config, goal, ob):
    """
    calculation final input with dynamic window
    """
    all_trajectory = []
    x_init = x[:]
    min_cost = float("inf")
    best_u = [0.0, 0.0]
    best_trajectory = np.array([x])
    # evaluate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1], config.v_resolution):
        for omega in np.arange(dw[2], dw[3], config.yaw_rate_resolution):

            trajectory = predict_trajectory(x_init, v, omega, config)
            all_trajectory.append(trajectory)
            # calc cost
            to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(trajectory, goal)
            speed_cost = config.speed_cost_gain * (config.max_speed - trajectory[-1, 3])
            ob_cost = config.obstacle_cost_gain * calc_obstacle_cost(trajectory, ob, config)

            final_cost = to_goal_cost + speed_cost + ob_cost

            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                best_u = [v, omega]
                best_trajectory = trajectory

    return best_u, best_trajectory, all_trajectory


def calc_obstacle_cost(trajectory, ob, config):
    """
        calc obstacle cost inf: collision
    """
    # map_reduce strategy failed....
    # vision_distance = 20
    # ob = np.zeros((1,2))
    # for i in range(len(ob_list)):
    #     if np.linalg.norm(ob_list[i] - trajectory[0,:2]) <= vision_distance:
    #         ob = np.vstack((ob, ob_list[i]))
    # ob = np.delete(ob, 0, axis=0)
    # print('ob shape is:', np.shape(ob))

    ox = ob[:, 0]  #(15,)
    oy = ob[:, 1]
    dx = trajectory[:, 0] - ox[:, None]  #因为障碍物和轨迹点数量不等,所以增加一列来表示
    # print(dx.shape)
    dy = trajectory[:, 1] - oy[:, None]
    rho = np.hypot(dx, dy)  #(15,21) 障碍和轨迹的欧式距离序列
    # print(rho)
    if config.robot_type == RobotType.rectangle:
        yaw = trajectory[:, 2]  #(21,)
        rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])  #(2,2,21)
        rot = np.transpose(rot, [2, 0, 1])  #(21,2,2)  轨迹的朝向序列

        local_ob = ob[:, None] - trajectory[:, 0:2] #(15,21,2) 综合了障碍和轨迹的坐标序列
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])   # (15*21, 2)
        local_ob = np.array([local_ob @ x for x in rot]) #(21,315,2)  对轨迹和障碍依次执行朝向变换
        local_ob = local_ob.reshape(-1, local_ob.shape[-1]) #(21*315, 2)  所有障碍物和轨迹点的经过旋转后的坐标

        upper_check = local_ob[:, 0] <= config.robot_length / 2  #
        right_check = local_ob[:, 1] <= config.robot_width / 2
        bottom_check = local_ob[:, 0] >= -config.robot_length / 2
        left_check = local_ob[:, 1] >= -config.robot_width / 2
        if (np.logical_and(np.logical_and(upper_check, right_check),
                           np.logical_and(bottom_check, left_check))).any():
            return float("Inf")
    elif config.robot_type == RobotType.circle:
        if np.array(rho <= config.robot_radius).any():
            return float("Inf")

    min_r = np.min(rho)
    # print(min_r)
    return 1.0 / min_r  # OK


def calc_to_goal_cost(trajectory, goal):
    """
        calc to goal cost with angle difference
    """

    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]
    error_angle = math.atan2(dy, dx)
    cost_angle = error_angle - trajectory[-1, 2]
    cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

    return cost


def plot_arrow(x, y, yaw, length=1.0, width=0.3):  # pragma: no cover
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)


def plot_robot(x, y, yaw, config):  # pragma: no cover
    if config.robot_type == RobotType.rectangle:
        outline = np.array([[-config.robot_length / 2, config.robot_length / 2,
                             (config.robot_length / 2), -config.robot_length / 2,
                             -config.robot_length / 2],
                            [config.robot_width / 2, config.robot_width / 2,
                             - config.robot_width / 2, -config.robot_width / 2,
                             config.robot_width / 2]])
        # Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
        #                  [-math.sin(yaw), math.cos(yaw)]])
        Rot1 = np.array([[math.cos(yaw), -math.sin(yaw)],
                         [math.sin(yaw), math.cos(yaw)]])
        # outline = (outline.T.dot(Rot1)).T
        outline = Rot1.dot(outline)
        outline[0, :] += x
        outline[1, :] += y
        plt.plot(np.array(outline[0, :]).flatten(),
                 np.array(outline[1, :]).flatten(), color='#3333CC', marker='o')
    elif config.robot_type == RobotType.circle:
        circle = plt.Circle((x, y), config.robot_radius, color="b")
        plt.gcf().gca().add_artist(circle)
        out_x, out_y = (np.array([x, y]) +
                        np.array([np.cos(yaw), np.sin(yaw)]) * config.robot_radius)
        plt.plot([x, out_x], [y, out_y], "-k")

def run_dwa():
    print(__file__ + " start!!")
    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    x = np.array([0.0, 0.0,0* math.pi , 0.0, 0.0])
    #goal position [x(m), y(m)]
    goal = np.array([10.0, 10.0])
    # obstacles [x(m) y(m), ....]
    ob = np.array([[-1, -1],
                    [-1.5, -1.5],
                    [-2, -2],
                    [0, 2],
                    [0.5, 2.5],
                    [4.0, 2.0],
                    [4.5, 2.0],
                    [5.0, 4.0],
                    [5.0, 4.5],
                    [5.0, 5.0],
                    [5.0, 6.0],
                    [5.0, 9.0],
                    [8.0, 9.0],
                    [7.0, 9.0],
                    [8.0, 10.0],
                    [9.0, 11.0],
                    [12.0, 13.0],
                    [12.0, 12.0],
                    [15.0, 15.0],
                    [13.0, 13.0]
                    ])

    robot_type = RobotType.rectangle
    # input [forward speed, yaw_rate]
    config = Config()
    config.robot_type = robot_type
    trajectory_ist = np.array(x)
    start = time.time()
    count =0
    while count ==0:
        count+=1
        # check reaching goal
        dist_to_goal = math.hypot(x[0] - goal[0], x[1] - goal[1])
        u_soll, trajectory_soll, all_trajectory = dwa_control(x, config, goal, ob)  # generate u = wheel_speed_soll
        # Embedded PI controller
        # u_total = K_p * (u_soll - u_ist) + K_I * (trajectory_soll[0] - trajectory_ist[0])
        # simulate robot
        #u_rpm = u_soll * 60/(2 * np.pi)
        # print('wheel speed is:( {:.1f} , {:.1f} )rpm'.format(u_rpm[0],u_rpm[1]))
        if np.linalg.norm(u_soll) < 2.0 and dist_to_goal >= config.robot_radius:
            u_soll = [0., 2.5* np.pi]
            print('deadzone checked')
        x = motion(x, u_soll, config.dt)  # simulate robot
        trajectory_ist = np.vstack((trajectory_ist, x))  # store state history

        if show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])

            plt.plot(x[0], x[1], "xr")
            plt.plot(goal[0], goal[1], "^r")
            plt.plot(ob[:, 0], ob[:, 1], "sk")
            for i in range(len(all_trajectory)):
                plt.plot(all_trajectory[i][:, 0], all_trajectory[i][:, 1], "-c")
            plt.plot(trajectory_soll[:, 0], trajectory_soll[:, 1], "-g")
            plot_robot(x[0], x[1], x[2], config)
            plot_arrow(x[0], x[1], x[2])
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.0001)

        if dist_to_goal <= config.robot_radius:
            print("Goal!!")
            break
    print('dwa process time:',time.time() - start)
    print("Done")
    if show_animation:
        plt.plot(trajectory_ist[:, 0], trajectory_ist[:, 1], "-r")
        plt.pause(0.0001)

    plt.show()


##########################################################################################
if __name__ == '__main__':
    run_dwa()
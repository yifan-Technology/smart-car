#!/usr/bin/python
# -*- coding: UTF-8 -*-

import matplotlib.pyplot as plt
import numpy as np
import time
import yaml


def parking_init():
    path = 'dwa_config.yaml'
    # path = '/home/yf/yifan/dwa_config.yaml'
    # path = "/home/taungdrier/Desktop/dwa_config.yaml"
    with open(path, "r") as f:
        config = yaml.load(f)
        print("load successful")

    car_state = config['Car_State']
    flag_state = config['Flag_State']
    return car_state, flag_state


class Parking_Controller():
    def __init__(self, car_state, flag_state):
        self.dt = car_state['dt']  # 0.2  # [s] Time tick for motion prediction
        self.to_goal_cost_gain = car_state['to_goal_cost_gain']  # 0.16
        # self.speed_cost_gain = car_state['speed_cost_gain'] # 0.394
        self.obstacle_cost_gain = car_state['obstacle_cost_gain']  # 0.6
        self.speed_adjust_param = car_state['speed_adjust_param']
        self.speed_cost_gain_max = car_state['speed_cost_gain_max']
        self.speed_cost_gain_min = car_state['speed_cost_gain_min']

        self.dist_to_goal = 1e10  # 1e10

        self.SHOW_ANIMATION = flag_state['SHOW_ANIMATION']  # True
        self.GOAL_ARRIVAED = flag_state['GOAL_ARRIVAED']  # False
        self.RESET_STATE = flag_state['RESET_STATE']  # False
        self.TEMPORARY_GOAL_ARRIVED = False

        self.m = 5
        self.g = 9.80665
        self.mu = 0.01
        self.x0 = 0.01
        self.a = 0.661 / 2
        self.b = 0.661 / 2
        self.c = 0.504 / 2  # 0.504/2
        self.h = 0.4
        self.r = 0.095
        self.Kp0 = car_state['Kp']
        self.Ki0 = car_state['Ki']
        self.Kp = 0
        self.Ki = 0
        self.v0 = car_state['v0']
        self.v = 0
        self.error_pre = 0
        self.goal_x = 0
        self.state = np.array([0.0, -0.3, np.pi / 2, 0.0, 0.0])
        self.vtrans = np.array([[1, -self.c], [1, self.c]]) / self.r
        self.robot_radius = 1.2  # [m] for collision check
        self.robot_width = 2 * self.c + 0.1  # [m] for collision check
        self.robot_length = self.a + self.b + 0.1  # [m] for collision check
        self.MovingState = "stop"

    def set_moving_state(self, moving_state):
        self.MovingState = moving_state

    def koordianten_transformation(self, wheel_speed, theta):
        omega_l = wheel_speed[0]
        omega_r = wheel_speed[1]

        vx = self.r * (omega_l + omega_r) / 2
        omega = self.r * (- omega_l + omega_r) / (2 * self.c)
        vy = -self.x0 * omega
        Rot = np.array([[np.cos(theta), -np.sin(theta)],
                        [np.sin(theta), np.cos(theta)]])
        v0 = np.array([vx, vy])
        inertial_velo = Rot @ v0

        return inertial_velo, vx, omega

    def speed_change(self, u_in, mode):
        speed_gain = 60 / (2 * np.pi) * 3591 / 187
        if mode == 'MOTOR_TO_PC':
            return u_in / speed_gain
        elif mode == 'PC_TO_MOTOR':
            return u_in * speed_gain

    def update_car_state(self, u):
        """
        motion model
        """
        self.state[2] += self.state[4] * self.dt
        velo, vx, omega = self.koordianten_transformation(u, self.state[2])
        self.state[0] += velo[0] * self.dt
        self.state[1] += velo[1] * self.dt
        self.state[3] = vx
        self.state[4] = omega

    def cal_perpendicular_dist(self,punkt1, punktm, punkt2):
        k12 = (punkt1[1] - punkt2[1])/(punkt1[0] - punkt2[0])
        d_den = np.sqrt(1 + 1/(k12**2))
        d_num = abs(punktm[0]/k12 + punktm[1] + 0.3)
        return d_num / d_den

    def get_current_error(self, error):
        if self.MovingState == "moving":
            currentError = np.arctan2(error[1], error[0])

        elif self.MovingState == "rotation":
            # currentError = np.pi / 2 - self.state[2]
            currentError = np.arctan2(error[1], error[0]) - np.pi/2
            print("get error:", currentError)

        elif self.MovingState == "forward":
            currentError = -error
        else:
            currentError = 0

        return currentError

    def pi_control(self, goal_error):
        current_error = self.get_current_error(goal_error)
        print("current error", current_error)
        if self.error_pre < 1e5:
            self.error_pre += current_error

        if self.MovingState == "moving":
            # omega = self.Kp * current_error * abs(self.goal_x) + self.Ki * self.error_pre
            omega = self.Kp * current_error + self.Ki * self.error_pre

        elif self.MovingState == "forward":
            omega = self.Kp * current_error

        elif self.MovingState == "rotation":
            omega = self.Kp * current_error + self.Ki * self.error_pre

            if abs(omega) > np.pi/2:
                omega = np.sign(omega) * np.pi/2
            if abs(omega) < np.pi/4:
                omega = np.sign(omega) * np.pi * 89/360
            
        u_control = np.array([self.v, omega])
        return u_control

    def parking_control(self, motor_ist, goal, flag):
        state = np.copy(self.state)
        u_ist = self.speed_change(motor_ist, 'MOTOR_TO_PC')
        _, vx_ist, omega_ist = self.koordianten_transformation(u_ist, state[2])
        # print('input speed is',u_ist)
        # # calculate designed goal
        # mittelpkt = np.array([(zw_goal[0] + zw_goal[2]) / 2, (zw_goal[1] + zw_goal[3]) / 2])
        # theta_m = np.arctan2(mittelpkt[1], mittelpkt[0])
        # goal = goal_d * np.array([np.cos(theta_m), np.sin(theta_m)]) + mittelpkt
        goal_left = goal[:2]
        goal_center = goal[2:4]
        goal_right = goal[4:6]
        dist = self.cal_perpendicular_dist(goal_left, goal_center, goal_right)
        norm_left = np.linalg.norm(goal_left)
        norm_right = np.linalg.norm(goal_right)

        error_theta_center = (np.arctan2(goal_center[1], goal_center[0] - 0.3) - np.pi/2) * 180/ np.pi
        # print("error theta:", error_theta_center)

        if self.RESET_STATE:
            state[:3] = np.array([0., 0., np.pi / 2])
            state[3:] = np.array([vx_ist, omega_ist])

        if self.MovingState == "stop":
            self.error_pre = 0
            u_cal = np.array([0., 0.])
            # bool1 = ()
            # bool2 = (abs(goal_center[1]) > 1.0)
            # booldist = (bool1 or bool2)
            if flag == 0 and abs(goal_center[0]) > 0.1:
                self.set_moving_state("moving")

            if flag == 1 or flag == 2 or abs(error_theta_center) > 5:
            # if flag == 1 or flag == 2:
                self.error_pre = 0
                self.set_moving_state("rotation")

            if flag == 3:
                self.set_moving_state("forward")

        if self.MovingState == "moving":
            # if norm_left > norm_right:
            #     target = goal_left
            # else:
            #     target = goal_right

            # error = goal_left - goal_right
            # bool1 = (abs(goal_center[0]) <= 0.1)
            # bool2 = (abs(goal_center[1]) < 1.0)
            # booldist = (bool1 or bool2)
            self.goal_x = goal_center[0]
            if flag == 1 or flag == 2 or abs(goal_center[0]) <= 0.1:
                u_cal = np.array([0., 0.])
                self.set_moving_state("stop")
            else:
                self.Kp = self.Kp0
                self.Ki = self.Ki0
                self.v = self.v0
                # u_cal = self.pi_control(target)
                # print("u cal", u_cal)
                u_cal = np.array([0.2, 0.])

        if self.MovingState == "rotation":

            if flag == 0:
                print("rot goal center:", goal_center)
                self.Kp = 0.8 * self.Kp0
                self.Ki = self.Ki0
                self.v = 0
                u_cal = self.pi_control(goal_center)
                print("int error:", self.error_pre)

            elif flag == 1:
                self.error_pre = 0
                u_cal = np.array([-0.05, np.pi * 30 / 100]) # turn left

            elif flag == 2:
                self.error_pre = 0
                u_cal = np.array([-0.05, -np.pi * 30 / 100]) # turn right

            if flag == 0 and abs(error_theta_center) < 5:
                u_cal = np.array([0, 0])
                self.set_moving_state("stop")

        if self.MovingState == "forward":
            bool1 = abs(goal_center[0]) <= 12
            bool2 = abs(goal_center[1]) >= 320
            boolm = bool1 and bool2
            if flag == 0 or flag == 1 or flag == 2 or boolm:
                u_cal = np.array([0., 0.])
                self.set_moving_state("final_stop")
            else:
                self.Kp = 0.09 * self.Kp0
                self.Ki = self.Ki0
                self.v = self.v0
                u_cal = self.pi_control(goal_center[0])
                # print("u cal", u_cal)
                # u_cal = np.array([0.2, 0.])

        if self.MovingState == "final_stop":
            self.error_pre = 0
            u_cal = np.array([0., 0.])

        u_soll = self.vtrans @ u_cal
        motor_soll = self.speed_change(u_soll, 'PC_TO_MOTOR')

        # print(self.MovingState)
        self.update_car_state(u_ist)

        return motor_soll

    def plot_arrow(self, length=0.6, width=0.3):
        dist_head = self.a * np.array([np.cos(self.state[2]), np.sin(self.state[2])])  # pragma: no cover
        x = self.state[0] + dist_head[0]
        y = self.state[1] + dist_head[1]
        # print('current head position:({:.2f},{:.2f})'.format(x, y))
        plt.arrow(x, y, length * np.cos(self.state[2]), length * np.sin(self.state[2]),
                  head_length=width, head_width=width)
        plt.plot(x, y)

    def plot_robot(self, x, y, yaw):  # pragma: no cover
        # if self.robot_type == RobotType.rectangle:
        outline = np.array([[-self.robot_length / 2, self.robot_length / 2,
                             (self.robot_length / 2), -self.robot_length / 2,
                             -self.robot_length / 2],
                            [self.robot_width / 2, self.robot_width / 2,
                             - self.robot_width / 2, -self.robot_width / 2,
                             self.robot_width / 2]])
        # Rot1 = np.array([[np.cos(yaw), np.sin(yaw)],
        #                  [-np.sin(yaw), np.cos(yaw)]])
        Rot1 = np.array([[np.cos(yaw), -np.sin(yaw)],
                         [np.sin(yaw), np.cos(yaw)]])
        # outline = (outline.T.dot(Rot1)).T
        outline = Rot1.dot(outline)
        outline[0, :] += x
        outline[1, :] += y
        plt.plot(np.array(outline[0, :]).flatten(),
                 np.array(outline[1, :]).flatten(), color='#3333CC', marker='o')

    def show_animation(self, target, trajectory_ist):
        plt.cla()
        # figManager = plt.get_current_fig_manager()
        # figManager.window.showMaximized()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])

        plt.plot(self.state[0], self.state[1], "xr")
        self.plot_robot(self.state[0], self.state[1], self.state[2])
        self.plot_arrow()
        plt.plot(trajectory_ist[:, 0], trajectory_ist[:, 1], "-r")
        plt.plot(target[0], target[1], "^r")
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.1)


if __name__ == '__main__':

    car_zustand, flag_zustand = parking_init()
    test_parking = Parking_Controller(car_zustand, flag_zustand)
    trajectory_ist = test_parking.state
    print(" start!!")
    target0 = np.array([2., 1.3])
    tartget1 = np.array([2., 4.3])
    wheelspeed_ist = np.array([0., 0.])
    ws_soll = np.array([0., 0.])

    target = target0

    while True:
        wheelspeed_soll = test_parking.parking_control(wheelspeed_ist, target)
        wheelspeed_ist = wheelspeed_soll
        # print("motor:", wheelspeed_ist)
        trajectory_ist = np.vstack((trajectory_ist, test_parking.state))

        if np.linalg.norm(target - test_parking.state[:2]) < 0.5 and test_parking.ControlState == "stop":
            test_parking.TEMPORARY_GOAL_ARRIVED = True

        if test_parking.TEMPORARY_GOAL_ARRIVED:
            target = tartget1

        if test_parking.SHOW_ANIMATION:
            test_parking.show_animation()

    plt.show()
    print("Done")

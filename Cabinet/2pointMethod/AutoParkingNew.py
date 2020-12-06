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
        self.Kp0 = 0.05#car_state['Kp']
        self.Ki0 = 0.01#car_state['Ki']
        self.Kp = 0
        self.Ki = 0
        self.v0 = 0.2#car_state['v0']
        self.v = 0
        self.error_pre = 0
        self.goal_x = 0
        self.state = np.array([0.0, -0.3, np.pi / 2, 0.0, 0.0])
        self.vtrans = np.array([[1, -self.c], [1, self.c]]) / self.r
        self.robot_radius = 1.2  # [m] for collision check
        self.robot_width = 2 * self.c + 0.1  # [m] for collision check
        self.robot_length = self.a + self.b + 0.1  # [m] for collision check
        self.MovingState = "stop"
        self.phase = "Phase: approach"
        self.phase_count = 0

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

    def get_current_error(self, error):
        if self.MovingState == "moving":# or self.MovingState == "forward":
            currentError = np.arctan2(error[1], error[0])
        elif self.MovingState == "rotation":
            # currentError = np.pi / 2 - self.state[2]
            currentError = np.arctan2(error[1], error[0]) - np.pi/2

        elif self.MovingState == "forward":
            currentError = error
        else:
            currentError = 0

        return currentError

    def pi_control(self, goal):
        current_error = self.get_current_error(goal)
        print("current error", current_error)
        if self.error_pre < 1e2:
            self.error_pre += current_error

        omega = self.Kp * (current_error) #+ self.Ki * self.error_pre
        if self.MovingState == "rotation":
            omega = self.Kp * current_error + self.Ki * self.error_pre
        elif self.MovingState == "forward":
            omega = self.Kp * (-current_error) #+ self.Ki * self.error_pre


        if abs(omega) > np.pi * 20 / 100:
            omega = np.sign(omega) * np.pi * 20 / 100#

        if self.MovingState == "rotation":
            if abs(omega) < np.pi * 20 / 100:
                omega = np.sign(omega) * np.pi * 10 / 100
            
        u_control = np.array([self.v, omega])
        return u_control
        
    def curvature_control(self, goal_center):

        #safe_d = 4

        #d = np.linalg.norm(goal_center)

        #self.v = self.v0 * d / safe_d

        # x = a * y * y
        a = goal_center[0] / (goal_center[1] ** 2)

        y0 = 0.03

        r = ((1 + (2 * a * y0) ** 2 ) ** (1.5)) / (2 * a)

        omega = self.v / r
            
        u_control = np.array([self.v, omega])
        
        return u_control

    def cacualte_temp_goal(self, goal_center, goal_left, goal_right, d):

        k = (goal_right[1] - goal_left[1]) / (goal_right[0] - goal_left[0])

        x_1 = goal_center[0] + d / np.sqrt((-1/k) ** 2 + 1) 

        x_2 = goal_center[0] - d / np.sqrt((-1/k) ** 2 + 1)

        y_1 = (-1/k) *(x_1 - goal_center[0]) + goal_center[1]
 
        y_2 = (-1/k) *(x_2 - goal_center[0]) + goal_center[1]

        point_1 = np.array([x_1,y_1])

        point_2 = np.array([x_2,y_2])

        # print("point_1:", point_1)
        # print("point_2:", point_2)




        if np.linalg.norm(point_1) > np.linalg.norm(point_2):
            return point_2
        else:
            return point_1

    def parking_control(self, motor_ist, goal, flag):
        state = np.copy(self.state)
        u_ist = self.speed_change(motor_ist, 'MOTOR_TO_PC')
        _, vx_ist, omega_ist = self.koordianten_transformation(u_ist, state[2])

        goal_left = goal[:2]
        goal_center = goal[2:4]
        goal_right = goal[4:6]
        
        distance_left = np.linalg.norm(goal_left)
        distance_right = np.linalg.norm(goal_right)
        error_dis = distance_left - distance_right

        if error_dis > 0:
            goal_center_temp = goal_left
            print("center ist left")
        else:
            goal_center_temp = goal_right
            print("center ist right")

        
        error_theta_center_tmp = (np.arctan2(goal_center_temp[1], goal_center_temp[0]) - np.pi/2) * 180/ np.pi #-0.3
        error_theta_center = (np.arctan2(goal_center[1], goal_center[0]) - np.pi/2) * 180/ np.pi #-0.3
        distance_center_tmp = np.linalg.norm(goal_center_temp)
        distance_center = np.linalg.norm(goal_center)
        print("goal_center_temp:", goal_center_temp)
        print("error_theta_center_tmp:", error_theta_center_tmp)
        # print("dis temp:", np.linalg.norm(goal_center_temp))

        if self.RESET_STATE:
            state[:3] = np.array([0., 0., np.pi / 2])
            state[3:] = np.array([vx_ist, omega_ist])

        if self.MovingState == "stop":
            self.error_pre = 0
            u_cal = np.array([0., 0.])

            if flag == 0 and np.abs(distance_center_tmp) > 1.2:                #np.abs(distance_left - distance_right) > 0.05
                self.set_moving_state("moving")

            # if flag == 1 or flag == 2 or abs(error_theta_center) > 5:
            elif flag == 1 or flag == 2:
                self.error_pre = 0
                self.set_moving_state("rotation")
            elif flag == 0:
                #error_theta_center_tmp = (np.arctan2(goal_center_temp[1], goal_center_temp[0]) - np.pi/2) * 180/ np.pi
                if np.abs(error_theta_center) > 2:
                    self.error_pre = 0
                    self.set_moving_state("rotation")
            elif flag == 3:
                self.set_moving_state("forward")
                #self.set_moving_state("stop")
            if np.abs(distance_center) <= 1.2 and np.abs(error_theta_center) <= 3:
                print("wait next-----------------------------------------------------------------")
                self.phase_count += 1
                if self.phase_count > 50:
                    self.phase = "Phase: goin"
            else:
                self.phase = "Phase: approach"
                self.phase_count = 0

            # elif flag == 1 or error_theta_center >= 5:
            #     self.set_moving_state("left_rotation")
            #
            # elif flag == 2 or error_theta_center <= -5:
            #     self.set_moving_state("right_rotation")

        if self.MovingState == "moving":

            if flag == 1 or flag == 2 or np.abs(distance_center_tmp) <= 1.2:   #np.abs(goal_center[0]) <= 0.1
                u_cal = np.array([0., 0.])
                self.set_moving_state("stop")
            else:
                self.Kp = self.Kp0
                self.Ki = self.Ki0
                self.v = self.v0
                u_cal = self.pi_control(goal_center_temp)
                #u_cal = self.curvature_control(goal_center)
                #u_cal = np.array([self.v, 0.])
                # print("u cal", u_cal)

        if self.MovingState == "rotation":

            if flag == 0:
                self.Kp = 1 * self.Kp0
                self.Ki = 0.5 * self.Ki0
                self.v = 0#-self.v0/10
                u_cal = self.pi_control(goal_center)
                print("int error:",self.error_pre)

            elif flag == 1:
                self.error_pre = 0
                u_cal = np.array([0., 1.]) * np.pi * 10 / 100 # turn left

            elif flag == 2:
                self.error_pre = 0
                u_cal = np.array([0., -1.]) * np.pi * 10 / 100 # turn right

            if flag == 0 and abs(error_theta_center) <= 3:
                self.set_moving_state("stop")

        if self.MovingState == "forward":
            bool1 = abs(goal_center[0]) <= 12
            bool2 = abs(goal_center[1]) >= 320
            boolm = bool1 and bool2
            if flag == 0 or flag == 1 or flag == 2 or boolm:
                u_cal = np.array([0., 0.])
                self.set_moving_state("final_stop")
            else:
                #self.Kp = 0.01 * self.Kp0
                #self.Ki = 0.1 * self.Ki0
                #self.v = self.v0*0.5
                #u_cal = self.pi_control(goal_center[0]-12)
                if abs(goal_center[0]) <= 30:
                    u_cal = np.array([self.v0*0.5, 0.])
                else:
                    omega = -np.sign(goal_center[0]) * np.pi * 10 / 100#
                    u_cal = np.array([0, omega])


                # print("u cal", u_cal)
                # u_cal = np.array([0.2, 0.])

        if self.MovingState == "final_stop":
            self.error_pre = 0
            u_cal = np.array([0., 0.])

        print("u_cal:", u_cal)

        if np.isnan(u_cal[0]) or np.isnan(u_cal[1]):
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

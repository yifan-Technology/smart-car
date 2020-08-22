import math
from enum import Enum
import matplotlib.pyplot as plt
import numpy as np
import time
import rclpy
from std_msgs.msg import Int8MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dwa_automobile import *


class ObstacleMap:
    def __init__(self):
        self._nodeImg = rclpy.create_node('Image1')
        self.subImg = self._nodeImg.create_subscription(
            Int8MultiArray,
            '/zed2/zed_node/left/image_rect_color',
            self.img_callback,
            1)
        print('msg0')
        self.subImg  # prevent unused variable warning

        self._image_data = None

    def img_callback(self, msg):
        print('msg_callback is True')
        self._image_data = msg

    @property
    def nodeImg(self):
        return self._nodeImg

    @property
    def image_data(self):
        return self._image_data


class LeftCam():
    def __init__(self):
        self._nodeImg = rclpy.create_node('image')
        self.subImg = self._nodeImg.create_subscription(
            Image,
            '/zed2/zed_node/left/image_rect_color',
            self.img_callback,
            10)
        self.subImg  # prevent unused variable warning
        print('msg_left')
        self.bridge = CvBridge()
        self._image_data = None

    def img_callback(self, msg):
        self._image_data = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    @property
    def nodeImg(self):
        return self._nodeImg

    @property
    def image_data(self):
        return self._image_data

def main(gx=40.0, gy=70.0, robot_type=RobotType.circle):
    print(__file__ + " start!!")
    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    x = np.array([7.0, 7.0,0* math.pi , 0.0, 0.0])
    # goal position [x(m), y(m)]
    goal = np.array([gx, gy])
    # obstacles [x(m) y(m), ....]

    ob = np.load("obstacle_loc.npy")
    # input [forward speed, yaw_rate]
    config = Config()
    config.robot_type = robot_type
    trajectory_ist = np.array(x)
    start = time.time()
    while True:
        # check reaching goal
        dist_to_goal = math.hypot(x[0] - goal[0], x[1] - goal[1])
        u_soll, trajectory_soll, all_trajectory = dwa_control(x, config, goal, ob)  # generate u = wheel_speed_soll
        # Embedded PI controller
        # u_total = K_p * (u_soll - u_ist) + K_I * (trajectory_soll[0] - trajectory_ist[0])
        # simulate robot
        u_rpm = u_soll * 60/(2 * np.pi)
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

if __name__ == '__main__':
    rclpy.init(args=None)
    ob = ObstacleMap()
    left = LeftCam()
    while 1:
        rclpy.spin_once(left.nodeImg, timeout_sec=0.05)
        print(np.shape(left.image_data))

    # main(robot_type=RobotType.rectangle)
    # main(robot_type=RobotType.circle)
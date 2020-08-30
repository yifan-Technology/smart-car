"""
Mobile robot motion planning sample with Dynamic Window Approach
author: Atsushi Sakai (@Atsushi_twi), Göktuğ Karakaşlı TONGJUECEHEN
"""
import matplotlib.pyplot as plt
import numpy as np
import time


class DWA_Controller():
    def __init__(self):

        self.show_animation = True
        self.m = 5
        self.g = 9.80665
        self.mu = 0.01
        self.x0 = 0.01
        self.a = 0.661 / 2
        self.b = 0.661 / 2
        self.c = 0.504 / 2
        self.h = 0.4
        self.r = 0.095
        self.x = np.array([2.5, 0., np.pi / 2, 0.0, 0.0])

        # I = 2 / 3 * rho * h * (c * (b ** 3 + a ** 3) + (c ** 3 * (a + b)))
        # self.rho = 2 * 1000 #density
        # N14 = b / (2 * (a + b)) * m * g
        # N23 = a / (2 * (a + b)) * m * g
        # ks = 1
        # R_a = 1
        # L_a = 0.01
        # J = 1 / 12 * 0.4 * r ** 2
        self.max_speed = 0.3 # [m/s]
        self.min_speed = -0.3  # [m/s]
        self.max_yaw_rate = 360.0 * np.pi / 180.0  # [rad/s]
        self.max_accel = 5.0  # [m/ss]
        self.max_delta_yaw_rate = 540.0 * np.pi / 180.0  # [rad/ss]
        self.v_resolution = 0.2  # [m/s]
        self.yaw_rate_resolution = 15. * np.pi / 180.0  # [rad/s]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.predict_time = 0.5  # [s]  less and more flexible
        self.to_goal_cost_gain = 0.3
        self.speed_cost_gain = 0.394
        self.obstacle_cost_gain = 1.29
        self.dist_to_goal = 1e10
        self.GOAL_ARRIVAED = False
        self.RESET_STATE = False

        # Also used to check if goal is reached in both types
        self.robot_radius = 1.5  # [m] for collision check
        self.robot_width = 2 * self.c + 0.1  # [m] for collision check
        self.robot_length = self.a + self.b + 0.1  # [m] for collision check
        self.target_zone = 1.2

    def obmap2coordinaten(self, obmap, res):
       return np.argwhere(obmap == 1) * res

    def koordianten_transformation(self, wheel_speed, theta):
        omega_l = wheel_speed[0]
        omega_r = wheel_speed[1]

        vx = self.r * (omega_l + omega_r) / 2
        omega = self.r * (- omega_l + omega_r) / (2 * self.c)
        vy = -self.x0 * omega

        dX = np.cos(theta) * vx - np.sin(theta) * vy
        dY = np.sin(theta) * vx + np.cos(theta) * vy
        inertial_velo = np.array([dX, dY])

        return inertial_velo, vx, omega

    def speed_change(self, u_in, mode):
        speed_gain = 60 / (2 * np.pi) * 3591/187
        if mode == 'MOTOR_TO_PC':
            return u_in / speed_gain
        elif mode == 'PC_TO_MOTOR':
            return u_in * speed_gain

    def dwa_control(self, motor_ist, x_pre, goal, obstacle, res):
        """
        Dynamic Window Approach control
        """
        state = np.copy(x_pre)
        # print('motor speed is',motor_ist)
        u_is = self.speed_change(motor_ist, 'MOTOR_TO_PC')
        # print('input speed is',u_ist)
        if self.RESET_STATE:
             state[:3] = np.array([2.5, 0., np.pi / 2])
             state[3:] = u_is

        dw = self.calc_dynamic_window(state)
        oblist = self.obmap2coordinaten(obstacle, res)
#        t1 = time.time()
        u_cal, traj_soll, all_traj = self.calc_control_and_trajectory(state, dw, goal, oblist)
        
        vtrans = np.array([[1, -self.c], [1, self.c]]) / self.r

        u_should = np.dot(vtrans, u_cal)
        # print('output speed untransformed is',u_soll)
        motor_soll = self.speed_change(u_should,'PC_TO_MOTOR')
        
        # check reaching goal
        dist_head = self.a * np.array([np.cos(state[2]), np.sin(state[2])])
        self.dist_to_goal = np.linalg.norm(dist_head+ state[:2] - goal)  # generate u = wheel_speed_soll

#        if np.linalg.norm(u_should) < 5.0 and self.dist_to_goal >= self.robot_radius:
#            motor_soll = np.array([400., -400.])
#            print('deadzone checked')

        if np.linalg.norm(u_is) < 5.0 and self.dist_to_goal <= self.target_zone:
            print("Goal!!")
            self.GOAL_ARRIVAED = True

        self.x = self.motion(self.x, u_is, self.dt)

        return motor_soll, traj_soll, all_traj



    def motion(self, state, u, dt):
        """
        motion model
        """

        state[2] += state[4] * dt
        velo, vx, omega = self.koordianten_transformation(u, state[2])

        state[0] += velo[0] * dt
        state[1] += velo[1] * dt
        state[3] = vx
        state[4] = omega

        return state

    def calc_dynamic_window(self, state):
        """
        calculation dynamic window based on current state x
        """
        # Dynamic window from robot specification
        Vs = [self.min_speed, self.max_speed,
              -self.max_yaw_rate, self.max_yaw_rate]

        # Dynamic window from motion model
        Vd = [state[3] - self.max_accel * self.dt,
              state[3] + self.max_accel * self.dt,
              state[4] - self.max_delta_yaw_rate * self.dt,
              state[4] + self.max_delta_yaw_rate * self.dt]

        #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
              max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

        return dw

    def predict_trajectory(self, x_init, v_soll, omega_soll):
        """
        predict trajectory with an input
        """
#        t1 = time.time()
        speed_soll = np.array([v_soll, omega_soll])
        x = np.copy(x_init)
        trajectory = np.copy(x)
        count_time = 0
        Vtrans = np.array([[1, -self.c], [1, self.c]]) / self.r
        wheel_speed = np.dot(Vtrans, speed_soll)
        # timeline = np.arange(self.dt,self.predict_time + self.dt, self.dt)
        # trajectory = self.motion_traj(x, wheel_speed, timeline)
        
        while count_time <= self.predict_time:  # now is 2s
            x = self.motion(x, wheel_speed, self.dt)
            trajectory = np.vstack((trajectory, x))
            count_time += self.dt
#        print('predic cal time',time.tim

#        print('traj num',trajectory.shape)
        return trajectory

    def calc_control_and_trajectory(self, state, dw, goal, obstacle):
        """
        calculation final input with dynamic window
        """
        all_traj = []
        x_init = np.copy(state)
        min_cost = float("inf")
        best_u = np.array([0.0, 0.0])
        best_traj = np.array(state)
        # evaluate all trajectory with sampled input in dynamic window
        for v in np.arange(dw[0], dw[1], self.v_resolution):
            for omega in np.arange(dw[2], dw[3], self.yaw_rate_resolution):
#                t1 = time.time()    
                traj = self.predict_trajectory(x_init, v, omega)
                all_traj.append(traj)
                # calc cost
                to_goal_cost = self.to_goal_cost_gain * self.calc_to_goal_cost(traj, goal)
                speed_cost = self.speed_cost_gain * (self.max_speed - traj[-1, 3])
                ob_cost = self.obstacle_cost_gain * self.calc_obstacle_cost(traj, obstacle)

                final_cost = to_goal_cost + speed_cost + ob_cost

                # search minimum trajectory
                if min_cost >= final_cost:
                    min_cost = final_cost
                    best_u = np.array([v, omega])
                    best_traj = traj
#                print('loop cal time', time.time()-t1)
#        print('v num', np.arange(dw[0], dw[1], self.v_resolution).shape)
#        print('omega num', np.arange(dw[2], dw[3], self.yaw_rate_resolution).shape)
        return best_u, best_traj, all_traj

    def calc_obstacle_cost(self, trajectory, obstacle):
        """
            calc obstacle cost inf: collision
        """
#        start = time.time()
        ox = obstacle[:, 0]  # (15,)
        oy = obstacle[:, 1]
        dx = trajectory[:, 0] - ox[:, None]  # 因为障碍物和轨迹点数量不等,所以增加一列来表示

        dy = trajectory[:, 1] - oy[:, None]
        rho = np.hypot(dx, dy)  # (15,21) 障碍和轨迹的欧式距离序列

        # if self.robot_type == RobotType.rectangle:
        yaw = trajectory[:, 2]  # (21,)
        rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])  # (2,2,21)
        rot = np.transpose(rot, [2, 0, 1])  # (21,2,2)  轨迹的朝向序列

        local_ob = obstacle[:, None] - trajectory[:, 0:2]  # (15,21,2) 综合了障碍和轨迹的坐标序列
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])  # (15*21, 2)
        local_ob = np.array([local_ob @ xvec for xvec in rot])  # (21,315,2)  对轨迹和障碍依次执行朝向变换
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])  # (21*315, 2)  所有障碍物和轨迹点的经过旋转后的坐标

        upper_check = local_ob[:, 0] <= self.robot_length / 2  #
        right_check = local_ob[:, 1] <= self.robot_width / 2
        bottom_check = local_ob[:, 0] >= -self.robot_length / 2
        left_check = local_ob[:, 1] >= -self.robot_width / 2
        if (np.logical_and(np.logical_and(upper_check, right_check),
                           np.logical_and(bottom_check, left_check))).any():
            return float("Inf")
        # elif self.robot_type == RobotType.circle:
        #    if np.array(rho <= self.robot_radius).any():
        #        return float("Inf")
        min_r = np.min(rho)
#        print('obstacle calcu time:',time.time()-start)
#        print('calcu points num:',local_ob.shape)
        # print(min_r)
        return 1.0 / min_r  # OK

    def calc_to_goal_cost(self, trajectory, goal):
        """
            calc to goal cost with angle difference
        """

        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]
        error_angle = np.arctan2(dy, dx)
        cost_angle = error_angle - trajectory[-1, 2]
        cost = abs(np.arctan2(np.sin(cost_angle), np.cos(cost_angle)))

        return cost


def human_obmap(goal, Obmap, scale):
    shape = np.shape(Obmap)
    cx = int(goal[0] / scale)
    cy = int(goal[1] / scale)
    testmap = np.zeros((shape[0], shape[1]))
    human_length = int(0.35 / scale)
    human_width = int(0.2 / scale)

    for i in np.linspace(0, 2 * np.pi, 100):
        x = (cx + human_length * np.cos(i)).astype(np.uint8)
        y = (cy +human_width + human_width * np.sin(i)).astype(np.uint8)
        testmap[x][y] = 1
    Obmap += testmap.astype(np.uint8)
    return Obmap


def plot_arrow(x, y, yaw, length=1.0, width=0.3):  # pragma: no cover
    plt.arrow(x, y, length * np.cos(yaw), length * np.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)


def plot_robot(x, y, yaw, dwa_config):  # pragma: no cover
    # if self.robot_type == RobotType.rectangle:
    outline = np.array([[-dwa_config.robot_length / 2, dwa_config.robot_length / 2,
                         (dwa_config.robot_length / 2), -dwa_config.robot_length / 2,
                         -dwa_config.robot_length / 2],
                        [dwa_config.robot_width / 2, dwa_config.robot_width / 2,
                         - dwa_config.robot_width / 2, -dwa_config.robot_width / 2,
                         dwa_config.robot_width / 2]])
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
    # elif self.robot_type == RobotType.circle:
    #    circle = plt.Circle((x, y), self.robot_radius, color="b")
    #    plt.gcf().gca().add_artist(circle)
    #    out_x, out_y = (np.array([x, y]) +
    #                    np.array([np.cos(yaw), np.sin(yaw)]) * self.robot_radius)
    #    plt.plot([x, out_x], [y, out_y], "-k")


if __name__ == '__main__':
    target = np.array([2.5, 4.0])
    # ob = np.array([[-1, -1],
    #                [-1.5, -1.5],
    #                [-2, -2],
    #                [0, 2],
    #                [0.5, 2.5],
    #                [4.0, 2.0],
    #                [4.5, 2.0],
    #                [5.0, 4.0],
    #                [5.0, 4.5],
    #                [5.0, 5.0],
    #                [5.0, 6.0],
    #                [5.0, 9.0],
    #                [8.0, 9.0],
    #                [7.0, 9.0],
    #                [8.0, 10.0],
    #                [9.0, 11.0],obstacle_loc
    #                [12.0, 13.0],
    #                [12.0, 12.0],
    #                [15.0, 15.0],
    #                [13.0, 13.0]
    #                ])
    obmap = np.load('test_map.npy')

    test_dwa = DWA_Controller()
    trajectory_ist = test_dwa.x
    print(" start!!")

    map_range = 5.  #[m/s]
    map_pixel = 100
    resolution = map_range / map_pixel
#    ob = human_obmap(target, obmap, resolution)
    ob = obmap
    wheelspeed_ist = np.array([0., 0.])
    # test_dwa.RESET_STATE = True
    # test_dwa.show_animation = False
    while True:
#            u_soll[1] = float(int(u_soll[1]))
        start = time.time()
        wheelspeed_soll, trajectory_soll, all_trajectory = test_dwa.dwa_control(wheelspeed_ist, test_dwa.x, target, ob,
                                                                               resolution)
        # input ist_wheel_speed and set test_dwa.RESET_STATE = True

        print('current wheel speed soll ', wheelspeed_soll)
        wheelspeed_ist = wheelspeed_soll
#            u_soll[1] = float(int(u_soll[1]))
        trajectory_ist = np.vstack((trajectory_ist, test_dwa.x))
        obmap_display = test_dwa.obmap2coordinaten(ob, resolution)
        print('elasped time:',time.time()- start)
        if test_dwa.show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])

            plt.plot(test_dwa.x[0], test_dwa.x[1], "xr")
            plt.plot(obmap_display[:, 0], obmap_display[:, 1], "sk")
            for i in range(len(all_trajectory)):
                plt.plot(all_trajectory[i][:, 0], all_trajectory[i][:, 1], "-c")
            print('traj num:',np.shape(all_trajectory))
            plt.plot(trajectory_soll[:, 0], trajectory_soll[:, 1], "-g")
            plot_robot(test_dwa.x[0], test_dwa.x[1], test_dwa.x[2], test_dwa)
            plot_arrow(test_dwa.x[0], test_dwa.x[1], test_dwa.x[2])
            plt.plot(target[0], target[1], "^r")
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.0001)

        if test_dwa.GOAL_ARRIVAED:
            break
    print("Done")
    print('dwa process time:', time.time() - start)
    if test_dwa.show_animation:
        plt.plot(trajectory_ist[:, 0], trajectory_ist[:, 1], "-r")
        plt.pause(0.0001)

    plt.show()

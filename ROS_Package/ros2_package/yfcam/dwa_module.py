#!/usr/bin/python
# -*- coding: UTF-8 -*-

import matplotlib.pyplot as plt
import numpy as np
import time
import yaml
import potential_field_planning


def dwa_init():
    #path = 'dwa_config.yaml'
    path = '/home/yf/yifan/dwa_config.yaml'
    # path = "/home/taungdrier/Desktop/dwa_config.yaml"
    with open(path, "r") as f:
        config = yaml.load(f)
        print("load successful")

    car_state = config['Car_State']
    flag_state = config['Flag_State']
    return car_state, flag_state


class DWA_Controller():
    def __init__(self, car_state, flag_state):

        self.max_speed = car_state['max_speed']  # 0.8  # [m/s]
        self.min_speed = car_state['min_speed']  # -0.8  # [m/s]
        self.max_yaw_rate = car_state['max_yaw_rate'] * np.pi / 180  # 180.0 * np.pi / 180.0  # [rad/s]
        self.max_accel = car_state['max_accel']  # 4.0  # [m/ss]
        self.max_delta_yaw_rate = car_state['max_delta_yaw_rate'] * np.pi / 180  # 360.0 * np.pi / 180.0  # [rad/ss]
        self.v_resolution = car_state['v_resolution']  # 0.2  # [m/s]
        self.yaw_rate_resolution = car_state['yaw_rate_resolution'] * np.pi / 180  # 15. * np.pi / 180.0  # [rad/s]
        self.min_wheel_speed = car_state['min_wheel_speed']  # [rpm]
        self.dt = car_state['dt']  # 0.2  # [s] Time tick for motion prediction
        self.predict_time = car_state['predict_time']  # 0.8  # [s]  less and more flexible

        self.to_goal_cost_gain = car_state['to_goal_cost_gain']  # 0.16
        # self.speed_cost_gain = car_state['speed_cost_gain'] # 0.394
        self.obstacle_cost_gain = car_state['obstacle_cost_gain']  # 0.6
        self.speed_adjust_param = car_state['speed_adjust_param']
        self.speed_cost_gain_max = car_state['speed_cost_gain_max']
        self.speed_cost_gain_min = car_state['speed_cost_gain_min']

        self.dist_to_goal = car_state['dist_to_goal']  # 1e10
        self.dead_count = car_state['dead_count']  # 0

        self.SHOW_ANIMATION = flag_state['SHOW_ANIMATION']  # True
        self.GOAL_ARRIVAED = flag_state['GOAL_ARRIVAED']  # False
        self.RESET_STATE = flag_state['RESET_STATE']  # False
        self.HUMAN_SHAPE = flag_state['HUMAN_SHAPE']  # False
        self.MAP_TO_OBCOORD = flag_state['MAP_TO_OBCOORD']  # True
        self.MEASURE_TIME = flag_state['MEASURE_TIME']  # False
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

        self.x = np.array([2.5, -0.3, np.pi / 2, 0.0, 0.0])
        self.robot_radius = 1.2  # [m] for collision check
        self.robot_width = 2 * self.c + 0.1  # [m] for collision check
        self.robot_length = self.a + self.b + 0.1  # [m] for collision check
        self.goal_range_1 = 10.
        self.goal_range_2 = 2.

    def dynamic_speed_cost(self, dmin):
        ds = self.speed_adjust_param * self.max_speed/self.max_accel
        if dmin <= ds:
            return self.speed_cost_gain_min + (self.speed_cost_gain_max - self.speed_cost_gain_min) * (dmin/ds)**1.8
        else:
            return self.speed_cost_gain_max

    def obmap2coordinaten(self, obmap, res):
        return np.argwhere(obmap == 1) * res

    def inverse_transforamtion(self, goal, ob_cood, tr_map=True, tr_goal_chain=False):
        theta = self.x[2] - np.pi / 2
        Rot_i2b = np.array([[np.cos(theta), np.sin(theta)],
                            [-np.sin(theta), np.cos(theta)]])
        if tr_goal_chain:
            goal_i = goal - self.x[:2]
            goal_b = (Rot_i2b.dot(goal_i.T) + np.array([[2.5], [0.]])).T

        else:
            goal_i = goal - self.x[:2]
            goal_b = Rot_i2b.dot(goal_i) + np.array([2.5, 0.])
            # print("goal:", goal_b)

        if tr_map:
            ob_i = ob_cood[:, :2] - self.x[:2]
            ob_b = (Rot_i2b.dot(ob_i.T) + np.array([[2.5], [0.]])).T
            return goal_b, ob_b
        else:
            return goal_b

    def clear_human_shape(self, goal, obstacle, res):
        goal_inmap = np.array((goal / res), dtype=int)
        x = goal_inmap[0]
        y = goal_inmap[1]
        xlim_l = np.clip(x - int(1 / res), 0, 99)
        xlim_r = np.clip(x + int(1 / res), 0, 99)
        ylim_l = np.clip(y - int(1 / res), 0, 99)
        ylim_r = np.clip(y + int(1 / res), 0, 99)
        if x - int(0.5 / res) >= 0 and y + int(0.5 / res) < 99:
            obstacle[xlim_l:xlim_r, ylim_l:ylim_r] = 0
        return obstacle

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
        speed_gain = 60 / (2 * np.pi) * 3591 / 187
        if mode == 'MOTOR_TO_PC':
            return u_in / speed_gain
        elif mode == 'PC_TO_MOTOR':
            return u_in * speed_gain

    def dwa_control(self, motor_ist, x_pre, zw_goal, ob_list):
        """
        Dynamic Window Approach control
        """
        state = np.copy(x_pre)
        # print('motor speed is',motor_ist)
        u_ist = self.speed_change(motor_ist, 'MOTOR_TO_PC')
        _, vx_ist, omega_ist = self.koordianten_transformation(u_ist, state[2])
        # print('input speed is',u_ist)

        if self.RESET_STATE:
            state[:3] = np.array([2.5, -0.3, np.pi / 2])
            state[3:] = np.array([vx_ist, omega_ist])

        dw = self.calc_dynamic_window(state)

        if self.HUMAN_SHAPE:
            ob_list = np.vstack((ob_list, human_obmap(target)))

        u_cal, traj_soll, all_traj = self.calc_control_and_trajectory(state, dw, zw_goal, ob_list)
        vtrans = np.array([[1, -self.c], [1, self.c]]) / self.r
        u_soll = np.dot(vtrans, u_cal)
        motor_soll = self.speed_change(u_soll, 'PC_TO_MOTOR')

        # check reaching goal
        # dist_head = self.a * np.array([np.cos(state[2]), np.sin(state[2])])
        # self.dist_to_goal = np.linalg.norm(dist_head + state[:2] - zw_goal)  # generate u = wheel_speed_soll
        self.dist_to_goal = np.linalg.norm(state[:2] - zw_goal)  # generate u = wheel_speed_soll
        # if self.dist_to_goal >= self.goal_range_1:
        #     self.to_goal_cost_gain = 0.3
        #
        # elif self.dist_to_goal < self.goal_range_1 and self.dist_to_goal >= self.goal_range_2:
        #     self.to_goal_cost_gain = 0.21
        # else:
        #     self.to_goal_cost_gain = 0.6

        # print('cost param', self.to_goal_cost_gain)
        if np.linalg.norm(motor_soll) < 1.414 * self.min_wheel_speed and self.dist_to_goal >= self.robot_radius:
            m_left = self.min_wheel_speed * np.sign(motor_soll[0])
            m_right = self.min_wheel_speed * np.sign(motor_soll[1])
            motor_soll = np.array([m_left, m_right])
            # self.dead_count += 1
            # if self.dead_count % 10 == 0:
            #     np.random.shuffle(motor_soll)
            print('deadzone checked')

        if self.dist_to_goal <= self.robot_radius:
            self.TEMPORARY_GOAL_ARRIVED = True
            # self.GOAL_ARRIVED = True

        self.x = self.motion(state, u_ist, self.dt)

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

                traj = self.predict_trajectory(x_init, v, omega)
                all_traj.append(traj)
                # calc cost
                to_goal_cost = self.to_goal_cost_gain * self.calc_to_goal_cost(traj, goal)
                dist_square, dmin = self.calc_obstacle_cost(traj, obstacle)
                if dist_square != float("inf"):
                    ob_cost = self.obstacle_cost_gain * dist_square
                else:
                    ob_cost = float("inf")
                speed_cost = self.dynamic_speed_cost(dmin) * (self.max_speed - traj[-1, 3])
                # print('speed cost:', speed_cost)
                final_cost = to_goal_cost + speed_cost + ob_cost

                # search minimum trajectory
                if min_cost >= final_cost:
                    min_cost = final_cost
                    best_u = np.array([v, omega])
                    best_traj = traj

        return best_u, best_traj, all_traj

    def calc_obstacle_cost(self, trajectory, obstacle):
        """
            calc obstacle cost inf: collision
        """

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
            return float("Inf"), 0
        # elif self.robot_type == RobotType.circle:
        #    if np.array(rho <= self.robot_radius).any():
        #        return float("Inf")
        min_r = np.min(rho)
        return 1.0 / min_r ** 2 , min_r # OK

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


def human_obmap(goal):
    cx = int(goal[0])
    cy = int(goal[1])
    human_length = 0.1
    human_width = 0.35
    i = np.linspace(0, 2 * np.pi, 20)
    x = (cx + human_length - human_length * np.cos(i))
    y = (cy + human_width - human_width * np.sin(i))
    human_shape = np.vstack((x, y)).T
    return human_shape

def sevenpoint_map(man):
    manx = man[0]
    many = man[1]
    point_map = np.array(
            [[manx - 0.4, many],
                [manx - 0.35, many],
              [manx - 0.3, many],
              [manx - 0.25, many],
                [manx - 0.2, many],
              [manx - 0.15, many],
            [manx - 0.1, many],
              [manx - 0.05, many],
              [manx , many],
              [manx + 0.05, many],
            [manx + 0.1, many],
              [manx + 0.15, many],
             [manx + 0.2, many],
              [manx + 0.25, many],
            [manx + 0.3, many],
              [manx + 0.35, many],
             [manx + 0.4, many]])
    return point_map

def plot_arrow(dwa, length=0.6, width=0.3):
    dist_head = dwa.a * np.array([np.cos(dwa.x[2]), np.sin(dwa.x[2])])  # pragma: no cover
    x = dwa.x[0] + dist_head[0]
    y = dwa.x[1] + dist_head[1]
    # print('current head position:({:.2f},{:.2f})'.format(x, y))
    plt.arrow(x, y, length * np.cos(dwa.x[2]), length * np.sin(dwa.x[2]),
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


def ob_to_obmap(obkood, res):
    map0 = np.zeros((500, 500))
    obkood += np.array([2., 2.])
    index = (obkood / res).astype(int)
    map0[index[:, 0], index[:, 1]] = 1
    return map0

if __name__ == '__main__':

    car_zustand, flag_zustand = dwa_init()
    test_dwa = DWA_Controller(car_zustand, flag_zustand)
    planner_state = potential_field_planning.pf_planner_init()
    pf_planner = potential_field_planning.Potential_Field_Planner(planner_state)
    trajectory_ist = test_dwa.x
    print(" start!!")
    target = np.array([2.5, 3.])
    map_range = 5.
    map_pixel = 100
    resolution = map_range / map_pixel
    test_map = False
    test_2m2m_map = True
    USE_SAVED_MAP = True

    test_dwa.TRANSFORM_MAP = True
    draw_initialframe_traj = False
    test_dwa.SHOW_ANIMATION = True
    test_dwa.MEASURE_TIME = True
    test_dwa.RESET_STATE = False
    # maplist = human_obmap(np.array([2.5, 2.]))
    maplist = sevenpoint_map(np.array([2.5, 2.]))
    np.save("test_2m2m_map.npy", maplist)
    # plt.plot(maplist[:, 0], maplist[:, 1], "sk")
    # plt.show()
    if USE_SAVED_MAP:
        # PATH ='test_map/obmap_square.npy'
        if test_2m2m_map:
            oblist = np.load('test_2m2m_map.npy')


        # obmap = np.load('test_map/obstacle_loc.npy')
        # obmap = test_dwa.clear_human_shape(target, obmap, resolution)
        # obmap[obmap >= 0.9] = 1
        # obmap[obmap < 0.9] = 0
        # oblist = test_dwa.obmap2coordinaten(obmap, resolution)
        # obmap = np.load('C:/Users/53276/OneDrive/Desktop/obmap.npy')[7]
        # obmap[obmap>50] = 1
        # obmap[obmap<=125] = 0
        # oblist = obmap
        # oblist = test_dwa.obmap2coordinaten(obmap, 5/30)
        # plt.scatter(oblist[:,0],oblist[:,1])
        # plt.show()
        elif test_map:
            obmap = np.load('C:/Users/53276/OneDrive/Desktop/test_map.npy')
            oblist = test_dwa.obmap2coordinaten(obmap, 5 / 100)

    else:
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
        oblist = ob

    wheelspeed_ist = np.array([0., 0.])
    ws_soll = np.array([0., 0.])
    goal_traj = np.copy(target)
    traj_ist = np.array([2.5, -0.3])
    # plt.plot(oblist[:, 0], oblist[:, 1], "sk")
    # plt.show()
    target_traj = pf_planner.potential_field_planning(target, oblist)
    i_ = 0
    # target_traj = np.array([[2.5,0.2],
    #                         [5.,2.],
    #                         [2.5,4.]])

    goal_to_reach = target_traj[0]
    if draw_initialframe_traj:
        test_dwa.RESET_STATE = False
        while not test_dwa.GOAL_ARRIVAED:
            ws_soll, traj_soll, all_traj = test_dwa.dwa_control(ws_soll, test_dwa.x, target, oblist)
            traj_ist = np.vstack((traj_ist, test_dwa.x[:2]))
            if np.linalg.norm(target - test_dwa.x[:2]) < 0.8:
                break
            print('state',test_dwa.x)
        print("generate soll traj")
        test_dwa.x = np.array([2.5, -0.3, np.pi / 2, 0, 0])
        wheelspeed_ist = np.array([0., 0.])
        test_dwa.GOAL_ARRIVAED = False
        test_dwa.RESET_STATE = True



    while True:
        start = time.time()
        wheelspeed_soll, trajectory_soll, all_trajectory = test_dwa.dwa_control(wheelspeed_ist, test_dwa.x,
                                                                                goal_to_reach,
                                                                                oblist)
        if i_ >= len(target_traj):
            goal_to_reach = target
        else:
            goal_to_reach = target_traj[i_]

        if test_dwa.TEMPORARY_GOAL_ARRIVED:
            i_ += 1
            test_dwa.TEMPORARY_GOAL_ARRIVED = False

        # print('distance:', np.linalg.norm(target - test_dwa.x[:2]))

        if test_dwa.RESET_STATE:
            # target_traj, oblist = test_dwa.inverse_transforamtion(target_traj, oblist, tr_goal_chain=True)
            goal_to_reach, oblist = test_dwa.inverse_transforamtion(goal_to_reach, oblist, tr_map=True)
            target_traj = test_dwa.inverse_transforamtion(target_traj, oblist, tr_map=False, tr_goal_chain=True)
            target = test_dwa.inverse_transforamtion(target, oblist, tr_map=False)
            goal_traj = np.vstack((goal_traj, target))


        if np.linalg.norm(target - test_dwa.x[:2]) < 1.:
            test_dwa.GOAL_ARRIVAED = True


        # print('current wheel speed soll ', wheelspeed_soll)
        wheelspeed_ist = wheelspeed_soll
        trajectory_ist = np.vstack((trajectory_ist, test_dwa.x))

        if test_dwa.HUMAN_SHAPE:
            oblist = np.vstack((oblist, human_obmap(target)))



        if test_dwa.SHOW_ANIMATION:
            plt.cla()
            # figManager = plt.get_current_fig_manager()
            # figManager.window.showMaximized()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])

            plt.plot(test_dwa.x[0], test_dwa.x[1], "xr")
            plt.plot(oblist[:, 0], oblist[:, 1], "sk")

            for i in range(len(all_trajectory)):
                plt.plot(all_trajectory[i][:, 0], all_trajectory[i][:, 1], "-c")
            # print(trajectory_soll)
            plt.plot(trajectory_soll[:, 0], trajectory_soll[:, 1], "-g")
            plot_robot(test_dwa.x[0], test_dwa.x[1], test_dwa.x[2], test_dwa)
            plot_arrow(test_dwa)

            if test_dwa.RESET_STATE:
                plt.plot(goal_traj[:, 0], goal_traj[:, 1], color='purple')
                # plt.plot(traj_ist[:, 0], traj_ist[:, 1], color='orange')
                plt.plot(target_traj[:, 0], target_traj[:, 1], color='orange')
            else:
                plt.plot(target_traj[:, 0], target_traj[:, 1], color='orange')
                plt.plot(trajectory_ist[:, 0], trajectory_ist[:, 1], "-r")

            plt.plot(target[0], target[1], "^r")
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.1)

        if test_dwa.MEASURE_TIME:
            print('fps:', 1/(time.time() - start))
        if test_dwa.GOAL_ARRIVAED:
            break

    print("Done")
    print('dwa process time:', time.time() - start)
    if test_dwa.SHOW_ANIMATION:
        plt.plot(trajectory_ist[:, 0], trajectory_ist[:, 1], "-r")
        plt.pause(0.0001)

    plt.show()
import numpy as np
import cv2
import copy
import rclpy
import yf_node
import dwa_module
import matplotlib.pyplot as plt
# import motorSerial
import time
import yaml
import potential_field_planning
import cv2


def init():
    # ???????
    global nodeName
    global mapSize
    # ??yaml??
    with open("/home/yf/yifan/config.yaml", "r") as f:
        config = yaml.load(f)

    mapSize = config["costMap"]["mapSize"]
    # ????????
    nodeName = config["RosTopic"]


def get_target(ziel):
    # ??????
    target = np.array([ziel.pose.position.x + 2.5 - 0.06, ziel.pose.position.z])
    return target


def wheel_speed_caculator(v_l_rpm, v_r_rpm):
    minimal = np.min(np.abs([v_l_rpm, v_r_rpm]))

    if minimal == 0 and np.max(np.abs([v_l_rpm, v_r_rpm])) == 0:
        wheel_speed = [v_l_rpm, v_r_rpm, v_l_rpm, v_r_rpm]
        print("cal 00000")
        return wheel_speed

    if minimal < 300:
        shift = 300 - minimal
        v_l_rpm += np.sign(v_l_rpm) * shift
        v_r_rpm += np.sign(v_r_rpm) * shift

    wheel_speed = [v_l_rpm, v_r_rpm, v_l_rpm, v_r_rpm]
    return wheel_speed


def main():
    # ??? ros2 python - rclpy & ??????
    rclpy.init()

    # ??????
    init()
    maps = yf_node.YF_Image_PY(nodeName["CostMap"], "CostMap", "sub")
    ziel = yf_node.YF_Goal(nodeName['Goal'], 'Goal', 'sub')
    real = yf_node.YF_RealSpeed(nodeName["RealSpeed"], "RealSpeed", "sub")
    soll = yf_node.YF_SollSpeed(nodeName["SollSpeed"], "SollSpeed", "pub")

    rclpy.spin_once(soll.node, timeout_sec=0.001)  # original 0.001
    rclpy.spin_once(real.node, timeout_sec=0.001)
    rclpy.spin_once(maps.node, timeout_sec=0.001)  # original 0.001
    rclpy.spin_once(ziel.node, timeout_sec=0.001)

    # ??DWA???
    test_own_map = False
    save_first_frame = True
    car_state, flag_state = dwa_module.dwa_init()
    pf_state = potential_field_planning.pf_planner_init()
    dwa = dwa_module.DWA_Controller(car_state, flag_state)
    pf_planner = potential_field_planning.Potential_Field_Planner(pf_state)
    old_target = np.array([-1., -1.])
    # init a time point for fps
    start = time.time()
    u_soll = np.array([0., 0., 0., 0.])
    car_traj = []
    # obmap_list = np.zeros((1, 30, 30))
    i_ = 0
    if test_own_map:
        target = np.array([2.5, 4.])
        oblist = np.load("/home/yf/dev_ws/test_2m2m_map.npy")

        target_traj = pf_planner.potential_field_planning(target, oblist)
        i_ = 0
        goal_to_reach = target_traj[0]
        car_traj = []
    try:
        while True:
            t = time.time()
            # time.sleep(0.1)

            rclpy.spin_once(soll.node, timeout_sec=0.001)  # original 0.001
            rclpy.spin_once(real.node, timeout_sec=0.001)
            rclpy.spin_once(maps.node, timeout_sec=0.001)  # original 0.001
            rclpy.spin_once(ziel.node, timeout_sec=0.001)
            # print('spin time :',time.time()-t)
            # print fps
            # print("fps: ", int(1 / (time.time() - t)))

            # target, oblist = dwa.inverse_transforamtion(target, oblist)

            if real.subMsg is None:
                print("Waiting for realspeed message")
                continue
            elif maps.subMsg is None:
                print("Waiting for map message")
                continue
            elif ziel.subMsg is None:
                print("Waiting for ziel message")
                continue
            elif np.sum(maps.subMsg) == 0:
                print("Map is None but given")
                continue
            else:

                # get target
                target = get_target(ziel.subMsg)

                # get wheel speed
                real_wheel = real.subMsg
                left_front = real_wheel[0]
                right_front = real_wheel[2]
                left_back = real_wheel[4]
                right_back = real_wheel[6]
                u_ist = np.array([(left_front + left_back) / 2, (right_front + right_back) / 2])

                # maps transformation
                obmap = maps.subMsg[:, :, 0]
                obmap[49,49] = 51
                # potential_map = cv2.resize(maps.subMsg, (20, 20), interpolation=cv2.INTER_CUBIC)
                obmap = np.swapaxes(obmap, 0, 1)
                obmap = np.flip(obmap, axis=1)
                # obmap = cv2.resize(obmap, (50, 50), interpolation=cv2.INTER_CUBIC)
                # if i_ < 500:
                #     obmap_list = np.concatenate((obmap_list, obmap[None, :]), axis=0)
                #     np.save('/home/yf/dev_ws/obmap.npy', obmap_list)
                #     i_ += 1

                obmap[obmap > 50] = 1
                oblist = dwa.obmap2coordinaten(obmap, 5. / 50.) + np.array([-0.06, 0.])

                # potential_map[potential_map > 0] = 1
                # ob_loc = pf_planner.obmap2coordinaten(potential_map, 5 / 20)
                # target_traj = pf_planner.potential_field_planning(target, ob_loc)  ########################################################change
                # goal_to_reach = target_traj[1]

                u_soll, traj_soll, all_traj = dwa.dwa_control(u_ist, dwa.x, target, oblist)

                # car_traj.append(dwa.x[:2])

                # if i_ >= len(target_traj):
                #     goal_to_reach = target
                # else:
                #     goal_to_reach = target_traj[i_]
                #
                # if dwa.TEMPORARY_GOAL_ARRIVED:
                #     i_ += 1
                #     dwa.TEMPORARY_GOAL_ARRIVED = False

                # goal_to_reach, oblist = dwa.inverse_transforamtion(goal_to_reach, oblist, tr_map=True)
                # target = dwa.inverse_transforamtion(target, oblist, tr_map=False)

                if dwa.SHOW_ANIMATION:
                    plt.cla()
                    figManager = plt.get_current_fig_manager()
                    figManager.window.showMaximized()
                    # for stopping simulation with the esc key.
                    plt.gcf().canvas.mpl_connect(
                        'key_release_event',
                        lambda event: [exit(0) if event.key == 'escape' else None])

                    # plt.plot(dwa.x[0], dwa.x[1], "xr")
                    plt.plot(oblist[:, 0], oblist[:, 1], "sk")

                    for i in range(len(all_traj)):
                        plt.plot(all_traj[i][:, 0], all_traj[i][:, 1], "-c")
                    # print(trajectory_soll)
                    plt.plot(traj_soll[:, 0], traj_soll[:, 1], "-g")
                    dwa_module.plot_robot(dwa.x[0], dwa.x[1], dwa.x[2], dwa)
                    dwa_module.plot_arrow(dwa)

                    # if dwa.RESET_STATE:
                        # plt.plot(goal_traj[:, 0], goal_traj[:, 1], color='purple')
                        # plt.plot(traj_ist[:, 0], traj_ist[:, 1], color='orange')
                        # plt.plot(target_traj[:, 0], target_traj[:, 1], color='orange')
                        # plt.plot(target[0], target[1], 'r')
                    # else:
                    #     plt.plot(traj_ist[:, 0], traj_ist[:, 1], "-r")

                    plt.plot(target[0], target[1], "^r")
                    plt.axis("equal")
                    plt.xlim(0,5)
                    plt.grid(True)
                    plt.pause(0.1)

                if np.linalg.norm(target - dwa.x[:2]) < 1.:
                    dwa.GOAL_ARRIVAED = True

                # speed infomation transforamtion
                u_soll[0] = float(int(u_soll[0]))
                u_soll[1] = float(int(u_soll[1]))
                wheel_speed = [u_soll[0], u_soll[1], u_soll[0], u_soll[1]]

                # if time.time() - start > 10:
                # np.save('/home/yf/yifan/car_traj.npy', np.array(car_traj))
                # dwa.GOAL_ARRIVAED = True

            if dwa.GOAL_ARRIVAED:
                print("goal arrived")
                wheel_speed = [0.0, 0.0, 0.0, 0.0]
                time.sleep(0.15)
                # soll.publishMsg(wheel_speed)
                # rclpy.spin_once(soll.node, timeout_sec=0.01)
                # dwa.GOAL_ARRIVAED = False

            print('fps = ', 1 / (time.time() - t))
            soll.publishMsg(wheel_speed)

            if dwa.GOAL_ARRIVAED:
                dwa.GOAL_ARRIVAED = False
                # time.sleep(0.3)
            # ??????

    finally:
        wheel_speed = [0.0, 0.0, 0.0, 0.0]
        soll.publishMsg(wheel_speed)
        rclpy.spin_once(soll.node, timeout_sec=0.001)
        soll.node.destroy_node()
        real.node.destroy_node()

    # plt.show()
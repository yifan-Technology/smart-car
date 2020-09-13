import numpy as np
import cv2
import copy
import rclpy
import yf_node
import node_test
import dwa_module
import matplotlib.pyplot as plt
# import motorSerial
import time
import yaml


def init():
    # 全局化节点名称
    global nodeName
    global mapSize
    # 读取yaml文件
    with open("/home/yf/yifan/config.yaml", "r") as f:
        config = yaml.load(f)

    mapSize = config["costMap"]["mapSize"]
    # 读取节点名称参数
    nodeName = config["RosTopic"]


def get_target(ziel):
    # 计算坐标位置
    target = np.array([ziel.pose.position.x + 2.5, ziel.pose.position.z])
    return target


def matplotlib_show(trajectory_soll, dwa, obmap, target, all_trajectory):
    # 是否用Matplotlib 展示地图
    # ！！！！！！！！！ 不可与cv2图片展示共存
    trajectory_ist = np.vstack((trajectory_soll, dwa.x))

    # for i in range(10000):
    #     if obMap[]
    obmap = dwa.obmap2coordinaten(obmap, 5 / 100)
    if dwa.show_animation:
        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])

        plt.plot(dwa.x[0], dwa.x[1], "xr")
        plt.plot(obmap[:, 0], obmap[:, 1], "sk")
        for i in range(len(all_trajectory)):
            plt.plot(all_trajectory[i][:, 0], all_trajectory[i][:, 1], "-c")
        # plt.plot(trajectory_soll[:, 0], trajectory_soll[:, 1], "-g")
        dwa_module.plot_robot(dwa.x[0], dwa.x[1], dwa.x[2], dwa)
        dwa_module.plot_arrow(dwa.x[0], dwa.x[1], dwa.x[2])
        plt.plot(trajectory_ist[:, 0], trajectory_ist[:, 1], "-r")
        plt.plot(target[0], target[1], "^r")
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.0001)
        # print('wheel speed is:',wheel_speed)
        print("Position :", dwa.x[:2])
        # plt.pause(1)
        plt.clf()
        plt.ioff()


def main():
    # 初始化 ros2 python - rclpy & 外置参数引入
    rclpy.init()

    # 构建相关节点
    init()
    maps = yf_node.YF_CostMap(nodeName["CostMap"], "CostMap")
    real = yf_node.YF_RealSpeed(nodeName["RealSpeed"], "RealSpeed")
    soll = yf_node.YF_SollSpeed(nodeName["SollSpeed"], "SollSpeed")
    ziel = yf_node.YF_Goal(nodeName['Goal'], 'Goal')
    flag = yf_node.YF_ObjectFlag(nodeName['ObjectFlag'], 'ObjectFlag')
    flag.publishMsg(101)

    # 广播节点的首次初始化
    rclpy.spin_once(soll.node, timeout_sec=0.05)  # original 0.001

    # 构建DWA控制类
    dwa = dwa_module.DWA_Controller()
    old_target = np.array([-1., -1.])
    # init a time point for fps
    t = time.time()
    # FAKE_TARGET = True
    # fake_target = np.array([2., 4.])
    while True:
        # 中断守护
        if cv2.waitKey(25) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break

        rclpy.spin_once(flag.node, timeout_sec=0.01)
        # subscribe flag signal
        signal = flag.subMsg.data

        if signal == 101:
            print("Waiting for a Target")
            continue
        if signal.tolist() > 0:
            # 刷新订阅的节点
            rclpy.spin_once(ziel.node, timeout_sec=0.01)
            rclpy.spin_once(maps.node, timeout_sec=0.01)
            rclpy.spin_once(real.node, timeout_sec=0.01)

            # 广播节点
            rclpy.spin_once(soll.node, timeout_sec=0.01)

            # 捕获数据
            obMap = maps.subMsg
            # print('map before',obMap)
            obMap = np.resize(obMap, (mapSize, mapSize))
            goal = ziel.subMsg

            # print fps
            print("fps: ", int(1 / (time.time() - t)))
            t = time.time()

            # 等待捕获所有信息
            if goal is None:
                print("Waiting for new target")
                continue
            else:
                if obMap is None:
                    print("Waiting for new obMap")
                    continue
                # print('current goal:', goal)
                if not signal == 100:
                    target = get_target(goal)
                    if target is not None:
                        print("target detected")
                        old_target = target
                        obMap = dwa.clear_human_shape(target, obMap, 5 / 100)
                        oblist = dwa.obmap2coordinaten(obMap, 5 / 100)
                    else:
                        continue
                    # if FAKE_TARGET:
                    #     target = fake_target
                    print('current target:', target)
                else:
                    obMap = dwa.clear_human_shape(old_target, obMap, 5 / 100)
                    oblist = dwa.obmap2coordinaten(obMap, 5 / 100)
                    old_target, oblist = dwa.inverse_transforamtion(old_target, oblist)
                    target = old_target

                # fake_target = np.copy(target)
                # print('using fake target')
                # else:
                #     # dwa.TRANSFORM_MAP = True
                # print('no transform goal')
                # print('no update obmap but transform frame')

            if real.subMsg is None:
                print("Waiting for real_speed and publish soll value [0,0,0,0]")
                soll.publishMsg([0.0, 0.0, 0.0, 0.0])
                continue
        else:
            continue

        # 捕获真实速度值
        real_wheel = real.subMsg
        # target = np.array([2.5,4.])
        # real_wheel = [200.,0,200.,0,200.,0,200.]
        # print("Got real_wheel: ",real_wheel)
        left_front = real_wheel[0]
        right_front = real_wheel[2]
        left_back = real_wheel[4]
        right_back = real_wheel[6]
        u_ist = np.array([(left_front + left_back) / 2, (right_front + right_back) / 2])


        dwa.GOAL_ARRIVAED = False
        dwa.RESET_STATE = True
        t1 = time.time()
        # print('oblist shape:',np.shape(oblist))
        u_soll, trajectory_soll, all_trajectory = dwa.dwa_control(u_ist, dwa.x, target, oblist)

        if dwa.MEASURE_TIME:
            t2 = time.time()
            print("run time:", t2 - t1)

        # speed infomation transforamtion
        u_soll[0] = float(int(u_soll[0]))
        u_soll[1] = float(int(u_soll[1]))
        wheel_speed = [u_soll[0], u_soll[1], u_soll[0], u_soll[1]]
        # # 使用Matplotlib展示地图
        # matplotlib_show(trajectory_soll,dwa,obMap,target,all_trajectory)
        if dwa.GOAL_ARRIVAED:
            wheel_speed = [0.0, 0.0, 0.0, 0.0]
            dwa.GOAL_ARRIVAED = False
        # print("wheel speed: ", wheel_speed)
        soll.publishMsg(wheel_speed)
        rclpy.spin_once(ziel.node, timeout_sec=0.01)

            # 杀死无用节点
    maps.node.destroy_node()
    soll.node.destroy_node()
    ziel.node.destroy_node()
    real.node.destroy_node()

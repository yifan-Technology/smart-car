#!/usr/bin/python
# -*- coding: UTF-8 -*-
"""
Potential Field based path planner
author: Atsushi Sakai (@Atsushi_twi)
Ref:
https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf
"""
import time
from collections import deque
import numpy as np
import yaml
import matplotlib.pyplot as plt


def pf_planner_init():
    # path = 'dwa_config.yaml'
    path = '/home/yf/yifan/dwa_config.yaml'
    # path = "/home/taungdrier/Desktop/dwa_config.yaml"
    with open(path, "r") as f:
        config = yaml.load(f)
        print("potential field planner load successful")

    planner_state = config['Planner_State']
    return planner_state


class Potential_Field_Planner():

    def __init__(self, planner_state):
        # Parameters
        self.KP = planner_state['KP']
        self.ETA = planner_state['ETA']
        self.AREA_WIDTH = planner_state['AREA_WIDTH']
        self.OSCILLATIONS_DETECTION_LENGTH = planner_state[
            'OSCILLATIONS_DETECTION_LENGTH']  # the number of previous positions used to check oscillations
        # Robot property
        self.start_loc = np.array([planner_state['start_locx'], planner_state['start_locy']])  # [m]
        self.ein_step = planner_state['ein_step']  # move step length [m]
        self.grid_size = planner_state['grid_size']  # potential grid size [m]
        self.robot_radius = planner_state['robot_radius']  # robot radius [m]
        # Flag state
        self.SHOW_ANIMATION = planner_state['SHOW_ANIMATION']

    def calc_potential_field(self, goal, ob_list):
        minx = -1.  # min(min(ob_list[:, 0]), self.start_loc[0], goal[0]) - self.AREA_WIDTH / 2.0
        miny = 0 - self.grid_size  # min(min(ob_list[:, 1]), self.start_loc[1], goal[1]) - self.AREA_WIDTH / 2.0
        maxx = 6  # max(max(ob_list[:, 0]), self.start_loc[0], goal[0]) + self.AREA_WIDTH / 2.0
        maxy = goal[1] + self.grid_size  # max(max(ob_list[:, 1]), self.start_loc[1], goal[1]) + self.AREA_WIDTH / 2.0
        xw = int(round((maxx - minx) / self.grid_size))
        yw = int(round((maxy - miny) / self.grid_size))

        # calc each potential
        pmap = [[0.0 for i in range(yw)] for i in range(xw)]

        for ix in range(xw):
            x = ix * self.grid_size + minx

            for iy in range(yw):
                y = iy * self.grid_size + miny
                ug = self.calc_attractive_potential(x, y, goal)
                uo = self.calc_repulsive_potential(x, y, ob_list)
                uf = ug + uo
                pmap[ix][iy] = uf

        return pmap, minx, miny

    def calc_attractive_potential(self, x, y, goal):
        return 0.5 * self.KP * np.hypot(x - goal[0], y - goal[1])

    def calc_repulsive_potential(self, x, y, ob_loc):
        # search nearest obstacle
        minid = -1
        dmin = float("inf")
        for i in range(len(ob_loc)):
            d = np.hypot(x - ob_loc[i, 0], y - ob_loc[i, 1])
            if dmin >= d:
                dmin = d
                minid = i

        # calc repulsive potential
        dq = np.hypot(x - ob_loc[minid, 0], y - ob_loc[minid, 1])

        if dq <= self.robot_radius:
            if dq <= 0.1:
                dq = 0.1

            return 0.5 * self.ETA * (1.0 / dq - 1.0 / self.robot_radius) ** 2
        else:
            return 0.0

    def get_motion_model(self):
        # dx, dy
        ein = self.ein_step
        motion = [[ein, 0],
                  [0, ein],
                  [-ein, 0],
                  [0, -ein],
                  [-ein, -ein],
                  [-ein, ein],
                  [ein, -ein],
                  [ein, ein]]

        return motion

    def oscillations_detection(self, previous_ids, ix, iy):
        previous_ids.append((ix, iy))

        if (len(previous_ids) > self.OSCILLATIONS_DETECTION_LENGTH):
            previous_ids.popleft()

        # check if contains any duplicates by copying into a set
        previous_ids_set = set()
        for index in previous_ids:
            if index in previous_ids_set:
                return True
            else:
                previous_ids_set.add(index)
        return False

    def potential_field_planning(self, goal_pos, ob_list):
        starttime = time.time()

        # calc potential field
        pmap, minx, miny = self.calc_potential_field(goal_pos, ob_list)

        # search path
        d = np.hypot(self.start_loc[0] - goal_pos[0], self.start_loc[1] - goal_pos[1])
        ix = round((self.start_loc[0] - minx) / self.grid_size)
        iy = round((self.start_loc[1] - miny) / self.grid_size)
        gix = round((goal_pos[0] - minx) / self.grid_size)
        giy = round((goal_pos[1] - miny) / self.grid_size)

        if self.SHOW_ANIMATION:
            self.draw_heatmap(pmap)
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(ix, iy, "*k")
            plt.plot(gix, giy, "*m")

        rx, ry = [self.start_loc[0]], [self.start_loc[1]]
        motion = self.get_motion_model()
        previous_ids = deque()

        while d >= self.grid_size:
            minp = float("inf")
            minix, miniy = -1, -1
            for i, _ in enumerate(motion):
                inx = int(ix + motion[i][0])
                iny = int(iy + motion[i][1])
                if inx >= len(pmap) or iny >= len(pmap[0]) or inx < 0 or iny < 0:
                    p = float("inf")  # outside area
                    print("outside potential!")
                else:
                    p = pmap[inx][iny]
                if minp > p:
                    minp = p
                    minix = inx
                    miniy = iny
            ix = minix
            iy = miniy
            xp = ix * self.grid_size + minx
            yp = iy * self.grid_size + miny
            d = np.hypot(goal_pos[0] - xp, goal_pos[1] - yp)
            rx.append(xp)
            ry.append(yp)

            if (self.oscillations_detection(previous_ids, ix, iy)):
                print("Oscillation detected at ({},{})!".format(ix, iy))
                break

            if self.SHOW_ANIMATION:
                plt.plot(ix, iy, ".r")
                plt.pause(0.001)
        print("process time :", time.time() - starttime)
        print("potential field planner Goal!!")
        solltraj = np.array(np.array([(x, y) for x, y in zip(rx, ry)]))
        return solltraj

    def draw_heatmap(self, data):
        data = np.array(data).T
        plt.pcolor(data, vmax=100.0, cmap=plt.cm.Blues)

    def obmap2coordinaten(self, obmap, res):
        return np.argwhere(obmap == 1) * res

    def compressmap(self, obbild, l_after):
        # 缩放比例
        # rate = max(obbild[0] / aftersize if obbild[0] > aftersize else 0, obbild[1] / aftersize if obbild[1] > aftersize else 0)
        l = len(obbild)
        scale = l / l_after
        core = np.zeros((l_after, l_after))

        for i in range(l):
            for j in range(l):
                core[int(i//scale), int(j//scale)] += obbild[i,j]
                # newmap.append(np.sum(core @ obbild[i:(scale + i), j:(scale + j)]) / scale)

        return core


if __name__ == '__main__':
    print(__file__ + " start!!")
    print("potential_field_planning start")

    ORIGINAL_MAP = False
    pf_state = pf_planner_init()
    pf_planner = Potential_Field_Planner(pf_state)
    pf_planner.SHOW_ANIMATION = True  # [m]
    goal_loc = np.array([2.5, 5.])  # [m]
    test_ideal_map = True
    test_newmap = False
    if ORIGINAL_MAP:
        ox = [15.0, 5.0, 20.0, 25.0]  # obstacle x position list [m]
        oy = [25.0, 15.0, 26.0, 25.0]  # obstacle y position list [m]
        ob_loc = np.array([(x, y) for x, y in zip(ox, oy)])
    else:
        # ob_loc = np.array([[-1, -1],
        #                    [-1.5, -1.5],
        #                    [-2, -2],
        #                    [0, 2],
        #                    [0.5, 2.5],
        #                    [4.0, 2.0],
        #                    [4.5, 2.0],
        #                    [5.0, 4.0],
        #                    [5.0, 4.5],
        #                    [5.0, 5.0],
        #                    [5.0, 6.0],
        #                    [5.0, 9.0],
        #                    [8.0, 9.0],
        #                    [7.0, 9.0],
        #                    [8.0, 10.0],
        #                    [9.0, 11.0],
        #                    [12.0, 13.0],
        #                    [12.0, 12.0],
        #                    [15.0, 15.0],
        #                    [13.0, 13.0]
        #                    ])
        if test_ideal_map:
            obmap = np.load('test_2m2m_map.npy')
            ob_loc = obmap

        elif test_newmap:
            obmap = np.load('obmap.npy')
            obmap[obmap <= 150] = 0
            obmap[obmap > 150] = 1

            obmap = pf_planner.compressmap(obmap, 10)
            obmap[obmap>0] =1
            ob_loc = pf_planner.obmap2coordinaten(obmap, 5 / 10)
        # obmap = np.flip(obmap, axis=1)
        # obmap = np.swapaxes(obmap, 1, 0)

        # obmap[obmap > 125] = 1
        # ob_loc = pf_planner.obmap2coordinaten(obmap, 5 / 50) + np.array([2.5, 2])

        plt.scatter(ob_loc[:, 0], ob_loc[:, 1])
        plt.xlim(0, 5)
        plt.ylim(0, 5)
        plt.show()

    if pf_planner.SHOW_ANIMATION:
        plt.grid(True)
        plt.axis("equal")

    # path generation
    soll_traj = pf_planner.potential_field_planning(goal_loc, ob_loc)

    if pf_planner.SHOW_ANIMATION:
        plt.show()
    # print(__file__ + " Done!!")

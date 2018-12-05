#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Dec  3 09:50:30 2018

@author: cngo
"""

"""

Mobile robot motion planning sample with Dynamic Window Approach

author: Atsushi Sakai (@Atsushi_twi)

"""

import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.table import Table
import matplotlib
show_animation = True


class Config():
    # simulation parameters

    def __init__(self,to_goal_cost_gain,robot_radius):
        # robot parameter
        self.max_speed = 1.0  # [m/s]
        self.min_speed = -0.5  # [m/s]
        self.max_yawrate = 40.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 0.2  # [m/ss]
        self.max_dyawrate = 40.0 * math.pi / 180.0  # [rad/ss]
        self.v_reso = 0.01  # [m/s] velocity resolution
        self.yawrate_reso = 0.1 * math.pi / 180.0  # [rad/s]
        self.dt = 0.1  # [s]
        self.predict_time = 3.0  # [s]
        self.to_goal_cost_gain = to_goal_cost_gain
        self.speed_cost_gain = 1.0
        self.robot_radius = robot_radius#1.0  # [m]


def motion(x, u, dt):
    # motion model

    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[2] += u[1] * dt
    x[3] = u[0]
    x[4] = u[1]

    return x


def calc_dynamic_window(x, config):

    # Dynamic window from robot specification
    Vs = [config.min_speed, config.max_speed,
          -config.max_yawrate, config.max_yawrate]

    # Dynamic window from motion model
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_dyawrate * config.dt,
          x[4] + config.max_dyawrate * config.dt]
    #  print(Vs, Vd)

    #  [vmin,vmax, yawrate min, yawrate max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
    #  print(dw)

    return dw


def calc_trajectory(xinit, v, y, config):

    x = np.array(xinit)
    traj = np.array(x)
    time = 0
    while time <= config.predict_time:
        x = motion(x, [v, y], config.dt)
        traj = np.vstack((traj, x))
        time += config.dt

    #  print(len(traj))
    return traj


def calc_final_input(x, u, dw, config, goal, ob, Ps):

    xinit = x[:]
    min_cost = 10000.0
    min_u = u
    min_u[0] = 0.0
    best_traj = np.array([x])

    # evalucate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1], config.v_reso):
        for y in np.arange(dw[2], dw[3], config.yawrate_reso):
            traj = calc_trajectory(xinit, v, y, config)

            # calc cost
            to_goal_cost = calc_to_goal_cost(traj, goal, config)
            speed_cost = config.speed_cost_gain * \
                (config.max_speed - traj[-1, 3])
            ob_cost = calc_obstacle_cost(traj, ob, config,Ps)
            #print(ob_cost)

            final_cost = to_goal_cost + speed_cost + ob_cost

            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                min_u = [v, y]
                best_traj = traj

    #  print(min_u)
    #  input()

    return min_u, best_traj, min_cost


def calc_obstacle_cost(traj, ob, config,Ps):
    # calc obstacle cost inf: collistion, 0:free

    skip_n = 2
    minr = float("inf")
    
    for ii in range(0, len(traj[:, 1]), skip_n):
        for i in range(len(ob[:, 0])):
            ox = ob[i, 0]
            oy = ob[i, 1]
            dx = traj[ii, 0] - ox
            dy = traj[ii, 1] - oy

            r = math.sqrt(dx**2 + dy**2)
            if r <= config.robot_radius:
                r_neg=Ps[i]/r
                #print(r_neg)
                return r_neg#float("Inf")  # collisiton

            if minr >= r:
                minr = r

    return 1.0 / minr  # OK


def calc_to_goal_cost(traj, goal, config):
    # calc to goal cost. It is 2D norm.

    dy = goal[0] - traj[-1, 0]
    dx = goal[1] - traj[-1, 1]
    goal_dis = math.sqrt(dx**2 + dy**2)
    cost = config.to_goal_cost_gain * goal_dis

    return cost


def dwa_control(x, u, config, goal, ob, Ps):
    # Dynamic Window control

    dw = calc_dynamic_window(x, config)

    u, traj, min_cost = calc_final_input(x, u, dw, config, goal, ob, Ps)

    return u, traj, min_cost


def plot_arrow(x, y, yaw, length=0.5, width=0.1):
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)

def main():
    
    print(__file__ + " start!!")
    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    x = np.array([0.0, 0.0, math.pi / 8.0, 0.0, 0.0])
    # goal position [x(m), y(m)]
    goal = np.array([15, 15])
    # obstacles [x(m) y(m), ....]
    ob = np.matrix([[0, 2],
                    [2.5, 8.0],
                    [4.0, 2.0],
                    [5.0, 4.0],
                    [5.0, 5.0],
                    [5.0, 6.0],
                    [5.0, 9.0],
                    [8.0, 9.0],
                    [7.0, 9.0],
                    [7.0, 6.2],
                    [12.0, 12.0],
                    [12.0, 14.0],
                    [13.0, 13.0]
                    ])
    
    u = np.array([0.0, 0.0])
#    max_speed = 1.0  # [m/s]
#    min_speed = -0.5  # [m/s]
#    max_yawrate = 40.0 * math.pi / 180.0  # [rad/s]
#    max_accel = 0.2  # [m/ss]
#    max_dyawrate = 40.0 * math.pi / 180.0  # [rad/ss]
#    v_reso = 0.01  # [m/s] velocity resolution
#    yawrate_reso = 0.1 * math.pi / 180.0  # [rad/s]
#    dt = 0.1  # [s]
#    predict_time = 3.0  # [s]
    to_goal_cost_gain = 1.0
#    speed_cost_gain = 1.0
    robot_radius = 1.0  # [m]
    config = Config(to_goal_cost_gain,robot_radius)
    traj = np.array(x)
    show_animation=False
    #Ps=np.random.rand(len(ob))*100
    Ps=([90.17962699, 18.89381439, 20.28729583, 93.90378764, 63.28318137,
           85.05038426, 29.05149061, 38.55092059, 87.28532153, 19.05553487,
           20.01487473, 40.46982651, 59.25035024])
    for i in range(1000):
        u, ltraj, min_cost = dwa_control(x, u, config, goal, ob, Ps)
        #determine cost of each movement including distance.. 
        x = motion(x, u, config.dt)
        
        traj = np.vstack((traj, x))  # store state history
        #if colision, stop run... add code here
        if show_animation:
    
            plt.cla()
            plt.plot(ltraj[:, 0], ltraj[:, 1], "-g")
            plt.plot(x[0], x[1], "xr")
            plt.plot(goal[0], goal[1], "xb")
            plt.plot(ob[:, 0], ob[:, 1], "ok")
            #plt.txt(ob[i,0],ob[i,1],Ps[i])
            plot_arrow(x[0], x[1], x[2])
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.0001)
    
        # check goal
        if math.sqrt((x[0] - goal[0])**2 + (x[1] - goal[1])**2) <= config.robot_radius:
            ob_a=np.array(ob)
            for i in range(len(ob)):
                x = ob_a[i,0]
                y = ob_a[i,1]
                plt.plot(x, y, 'bo')
                plt.text(x * (1 + 0.01), y * (1 + 0.01),np.round(Ps[i],2), fontsize=10)
            print("Goal!!")
            break
    show_animation=True
    print("Done")
    if show_animation:
        plt.plot(traj[:, 0], traj[:, 1], "-r")
        plt.grid(True)
        plt.plot(x[0], x[1], "xr")
        plt.plot(goal[0], goal[1], "xb")
        plt.show()
    

if __name__ == '__main__':
    main()

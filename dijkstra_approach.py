#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dijkstra grid based planning

author: Atsushi Sakai(@Atsushi_twi)

Created on Wed Dec  5 08:56:12 2018

edited: cngo
"""

import matplotlib.pyplot as plt
import math
import numpy as np

show_animation = True


class Node:

    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)

def calc_obstacle_cost(c_x , c_y, ob, rr, Ps):
    # calc obstacle cost inf: collistion, 0:free
    #rr = robot radius
    skip_n = 2
    minr = float("inf")
    
    for i in range(len(ob[:, 0])):
        ox = ob[i, 0]
        oy = ob[i, 1]
        dx = c_x - ox
        dy = c_y - oy

        r = math.sqrt(dx**2 + dy**2)
        if r <= rr:
            r_neg=Ps[i]/r
            #print(r_neg)
            return r_neg#float("Inf")  # collisiton

        if minr >= r:
            minr = r

    return 1.0 / minr  # OK

def calc_to_goal_cost(c_x,c_y, gx, gy, to_goal_cost_gain):
    # calc to goal cost. It is 2D norm.

    dy = gy - c_y
    dx = gx - c_x
    goal_dis = math.sqrt(dx**2 + dy**2)
    cost = to_goal_cost_gain * goal_dis

    return cost

def dijkstra_planning(sx, sy, gx, gy, ox, oy, reso, rr, Ps, ob,to_goal_cost_gain):
    """
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    reso: grid resolution [m]
    rr: robot radius[m]
    """

    nstart = Node(round(sx / reso), round(sy / reso), 0.0, -1)
    ngoal = Node(round(gx / reso), round(gy / reso), 0.0, -1)
    ox = [iox / reso for iox in ox]
    oy = [ioy / reso for ioy in oy]

    obmap, minx, miny, maxx, maxy, xw, yw = calc_obstacle_map(ox, oy, reso, rr, Ps)

    motion = get_motion_model()

    openset, closedset = dict(), dict()
    openset[calc_index(nstart, xw, minx, miny)] = nstart

    while 1:
        c_id = min(openset, key=lambda o: openset[o].cost)
        current = openset[c_id]
        #print("current", current)

        # show graph
#        if show_animation:
#            plt.plot(current.x * reso, current.y * reso, "xc")
#            #print(current.cost)
#            if len(closedset.keys()) % 10 == 0:
#                plt.pause(0.001)

        if current.x == ngoal.x and current.y == ngoal.y:
            print("Find goal")
            ngoal.pind = current.pind
            ngoal.cost = current.cost
            print(ngoal.cost)
            break

        # Remove the item from the open set
        del openset[c_id]
        # Add it to the closed set
        closedset[c_id] = current

        # expand search grid based on motion model
        for i in range(len(motion)):
            cost_ob=calc_obstacle_cost(current.x,current.y, ob, rr, Ps)
            cost_goal=calc_to_goal_cost(current.x,current.y, gx, gy, to_goal_cost_gain)
            #print(cost_ob)
            #cost_ob=0
            node = Node(current.x + motion[i][0], current.y + motion[i][1],
                        current.cost + motion[i][2]+cost_ob+cost_goal, c_id)
            n_id = calc_index(node, xw, minx, miny)

            if not verify_node(node, obmap, minx, miny, maxx, maxy):
                continue

            if n_id in closedset:
                continue
            # Otherwise if it is already in the open set
            if n_id in openset:
                if openset[n_id].cost > node.cost:
                    openset[n_id].cost = node.cost
                    openset[n_id].pind = c_id
            else:
                openset[n_id] = node

    rx, ry = calc_final_path(ngoal, closedset, reso)

    return rx, ry


def calc_final_path(ngoal, closedset, reso):
    # generate final course
    rx, ry = [ngoal.x * reso], [ngoal.y * reso]
    pind = ngoal.pind
    while pind != -1:
        n = closedset[pind]
        rx.append(n.x * reso)
        ry.append(n.y * reso)
        pind = n.pind

    return rx, ry


def verify_node(node, obmap, minx, miny, maxx, maxy):
    #print(node.x,node.y)
    if obmap[node.x][node.y]:
        return False

    if node.x < minx:
        return False
    elif node.y < miny:
        return False
    elif node.x > maxx:
        return False
    elif node.y > maxy:
        return False

    return True


def calc_obstacle_map(ox, oy, reso, vr, Ps):

    minx = 0#round(int(min(ox)))
    miny = 0#round(int(min(oy)))
    maxx = 15-1#round(int(max(ox)))
    maxy = 15-1#round(int(max(oy)))
    #  print("minx:", minx)
    #  print("miny:", miny)
    #  print("maxx:", maxx)
    #  print("maxy:", maxy)

    xwidth = 15-1#round(maxx - minx)
    ywidth = 15-1#round(maxy - miny)
    #  print("xwidth:", xwidth)
    #  print("ywidth:", ywidth)

#    # obstacle map generation
#    obmap = [[False for i in range(xwidth)] for i in range(ywidth)]
#    for ix in range(xwidth):
#        x = ix + minx
#        for iy in range(ywidth):
#            y = iy + miny
#            #  print(x, y)
#            for iox, ioy in zip(ox, oy):
#                d = math.sqrt((iox - x)**2 + (ioy - y)**2)
#                if d <= vr / reso:
#                    obmap[ix][iy] = True
#                    break
    data=np.zeros([16,16])
    #data=np.zeros([int(16/reso),int(16/reso)])

    for i in range(len(ox)):

        data[int(ox[i]),int(oy[i])]=Ps[i]
    obmap=data>0
    return obmap, minx, miny, maxx, maxy, xwidth, ywidth


def calc_index(node, xwidth, xmin, ymin):
    return (node.y - ymin) * xwidth + (node.x - xmin)


def get_motion_model():
    # dx, dy, cost
    motion = [[1, 0, 1],
              [0, 1, 1],
              [-1, 0, 1],
              [0, -1, 1],
              [-1, -1, math.sqrt(2)],
              [-1, 1, math.sqrt(2)],
              [1, -1, math.sqrt(2)],
              [1, 1, math.sqrt(2)]]

    return motion


def main(ob,Ps,to_goal_cost_gain):
    print(__file__ + " start!!")

    # start and goal position
    sx = 0  # [m]
    sy = 0  # [m]
    gx = 14#50.0  # [m]
    gy = 14#50.0  # [m]
    grid_size = 1.0  # [m]
    robot_size = 1.0  # [m]
    
#    to_goal_cost_gain=100
    ox = []
    oy = []
#    ob = np.matrix([[0, 2],
#                    [2.5, 8.0],
#                    [4.0, 2.0],
#                    [5.0, 4.0],
#                    [5.0, 5.0],
#                    [5.0, 6.0],
#                    [5.0, 9.0],
#                    [8.0, 9.0],
#                    [7.0, 9.0],
#                    [7.0, 6.2],
#                    [12.0, 12.0],
#                    [12.0, 14.0],
#                    [13.0, 13.0]
#                    ])
    ox=np.array(ob[:,0])
    oy=np.array(ob[:,1])
#    Ps=([90.17962699, 18.89381439, 20.28729583, 93.90378764, 63.28318137,
#       85.05038426, 29.05149061, 38.55092059, 87.28532153, 19.05553487,
#       20.01487473, 40.46982651, 59.25035024])
    if show_animation:
        ox=list(ox)
        oy=list(oy)
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "xr")
        plt.plot(gx, gy, "xb")
        ob_a=np.array(ob)
        for i in range(len(ob)):
            x = ob_a[i,0]
            y = ob_a[i,1]
            plt.plot(x, y, 'bo')
            plt.text(x * (1 + 0.01), y * (1 + 0.01),np.round(Ps[i],2), fontsize=10)
            plt.plot(sx, sy, "xr")


    rx, ry = dijkstra_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_size,Ps, ob, to_goal_cost_gain)

    if show_animation:
##        plt.plot(ox, oy, ".k")
##        plt.plot(sx, sy, "xr")
##        plt.plot(gx, gy, "xb")
##        plt.grid(True)
##        plt.axis("equal")
#
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "xr")
        plt.plot(gx, gy, "xb")
        ob_a=np.array(ob)
        for i in range(len(ob)):
            x = ob_a[i,0]
            y = ob_a[i,1]
            plt.plot(x, y, 'bo')
            plt.text(x * (1 + 0.01), y * (1 + 0.01),np.round(Ps[i],2), fontsize=10)
            plt.plot(sx, sy, "xr")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        #plt.axis("equal")
        plt.plot(rx, ry, "-r")
        plt.xlim([-1,15])
        plt.ylim([-1,15])
        plt.title('Dijkstra')
        plt.show()
    return rx, ry

if __name__ == '__main__':
    rx,ry=main(ob,Ps,to_goal_cost_gain)

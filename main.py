#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 16 11:52:54 2018

@author: cngo
"""
from math import sqrt
import pathv1 #Astar
import matplotlib.pyplot as plt
import Dynamic_Window
import dijkstra_approach
import numpy as np

def pathlength(x,y):
    n = len(x) 
    lv = [sqrt((x[i]-x[i-1])**2 + (y[i]-y[i-1])**2) for i in range (1,n)]
    L = sum(lv)
    return L
#%%
ob = np.matrix([[0, 2],
                [2.0, 8.0],
                [2.0, 2.0],
                [4.0, 2.0],
                [5.0, 4.0],
                [5.0, 5.0],
                [5.0, 6.0],
                [5.0, 9.0],
                [8.0, 8.0],
                [8.0, 9.0],
                [7.0, 9.0],
                [7.0, 6.2],
                [8.0, 5.0],
                [8.0, 12.0],
                [9.0, 8.0],
                [9.0, 2.0],
                [12.0, 6.0],
                [10.0, 10.0],
                [12.0, 12.0],
                [12.0, 14.0],
                [13.0, 13.0]
                ])
Ps=([0.80022382, 0.90780089, 0.38949302, 0.49433963, 0.42150666,
       0.53604437, 0.72230471, 0.21344193, 0.08727899, 0.3801174 ,
       0.51348767, 0.79567055, 0.15080252, 0.88542027, 0.00694119,
       0.63651462, 0.33937279, 0.56035994, 0.09873441, 0.08990051,
       0.68708375])
    
to_goal_cost_gain=1
robot_radius=1

#%%


l_dij=list()
l_dw =list()
goal_gain=list()
for i in range(5):
    to_goal_cost_gain=1*(i+1)
    #DIJKSTRA APPROACH
    print(to_goal_cost_gain)
    rx_dij,ry_dij=dijkstra_approach.main(ob,Ps,to_goal_cost_gain)
    traj = Dynamic_Window.main(ob,Ps,to_goal_cost_gain)
    
    rx_dw = traj[:,0]
    ry_dw = traj[:,1]
    
    l_dij.append([rx_dij,ry_dij])
    l_dw.append([rx_dw,ry_dw])
    goal_gain.append(to_goal_cost_gain)
    

#%%

fig1, ax1 = plt.subplots()
fig2, ax2 = plt.subplots()
ax1.cla()
ax1.cla()
Len_dij=np.zeros(len(l_dij))
Len_dw =np.zeros(len(l_dw))
'''plot routes'''
for i in range(len(l_dij)):
    dat_dij=l_dij[:][:][i]
    dat_dw =l_dw[:][:][i]
    rx_dij,ry_dij=dat_dij
    rx_dw,ry_dw=dat_dw
    
    Len_dij[i]=pathlength(rx_dij,ry_dij)
    Len_dij[i]=pathlength(rx_dw,ry_dw)
    leg=('goal to cost gain: %d' % goal_gain[i])
    
    ax1.plot(rx_dij,ry_dij,'-',label='gain: {%02d}'.format(d=goal_gain[i]))
    ax2.plot(rx_dw,ry_dw,'-')   
ax1.legend()
ax1.grid(True,which='both')
ax1.set_title('Dijkstra, Varying Goal to Cost Gain')

ax2.legend()
ax2.grid(True,which='both')
ax2.set_title('Dynamic Window Approach, Varying Goal to Cost Gain')
#ax1.legend()
ob_a=np.array(ob)
for i in range(len(ob)):
    xb = ob_a[i,0]
    yb = ob_a[i,1]
    ax1.plot(xb, yb, 'bo',markersize=9)
    ax1.text(xb * (1 + 0.01), yb * (1 + 0.01),np.round(Ps[i],2), fontsize=10)

    
#%%
    
    
#finalPath=pathv1.main(to_goal_cost_gain,robot_radius)
#Astar=np.array(finalPath)
#
#x=Astar[:,0]
#y=Astar[:,1]
#L_star=pathlength(x,y)
#print(L_star)
#
##plt.plot(traj[:, 0], traj[:, 1], "-r")

#plt.plot(x,y,"-r")
#plt.grid(True)
#plt.plot(0, 0, "xr")
#plt.plot(goal[0], goal[1], "xb")
#plt.show()

#Test different MDP alg

#Dijstar
#A*
#MC

#POMDP
#Online Particle
#MC

#Animation

#Compare
#traj = [x,y, u, dt]

#Astar_backup=Astar #when cost of 5 and 40
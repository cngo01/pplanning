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

def pathlength(x,y):
    n = len(x) 
    lv = [sqrt((x[i]-x[i-1])**2 + (y[i]-y[i-1])**2) for i in range (1,n)]
    L = sum(lv)
    return L
#%%
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
Ps=([90.17962699, 18.89381439, 20.28729583, 93.90378764, 63.28318137,
       85.05038426, 29.05149061, 38.55092059, 87.28532153, 19.05553487,
       20.01487473, 40.46982651, 59.25035024])
    
to_goal_cost_gain=1
robot_radius=2
finalPath=pathv1.main(to_goal_cost_gain,robot_radius)
Astar=np.array(finalPath)

x=Astar[:,0]
y=Astar[:,1]
L_star=pathlength(x,y)
print(L_star)

#plt.plot(traj[:, 0], traj[:, 1], "-r")
ob_a=np.array(ob)
for i in range(len(ob)):
    xb = ob_a[i,0]
    yb = ob_a[i,1]
    plt.plot(xb, yb, 'bo')
    plt.text(xb * (1 + 0.01), yb * (1 + 0.01),np.round(Ps[i],2), fontsize=10)
plt.plot(x,y,"-r")
plt.grid(True)
plt.plot(0, 0, "xr")
plt.plot(goal[0], goal[1], "xb")
plt.show()

#%% Dynamic Window


#x = traj[:,0]#[pts[i][0] for i in range(n)]
#y = traj[:,1]#[pts[i][1] for i in range(n)]

# Now call the function with the x and y 
# and print the result
#l = pathlength(x, y)
#print(l)
#generate probability map

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
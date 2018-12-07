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
import pandas

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
                [10.0, 9.0],
                [11.0, 9.0],
                [12.0, 6.0],
                [10.0, 10.0],
                [12.0, 12.0],
                [12.0, 14.0],
                [13.0, 13.0]
                ])
    
Ps_p=([0.18822291, 0.05969734, 0.74069164, 0.47933208, 0.48850482,
   0.02146035, 0.31580583, 0.3587886 , 0.79536192, 0.97236535,
   0.15689085, 0.10305778, 0.39755615, 0.67396048, 0.89009032,
   0.10704731, 0.70899104, 0.13591165, 0.79673235, 0.27865084,
   0.29617043, 0.85726783, 0.66711286])
    
Ps=([18.82229137,  5.96973403, 74.06916379, 47.93320819, 48.85048176,
        2.14603484, 31.5805826 , 35.87885955, 79.53619248, 97.23653457,
       15.68908499, 10.30577757, 39.7556145 , 67.39604789, 89.00903243,
       10.70473092, 70.89910441, 13.59116515, 79.67323489, 27.86508364,
       29.61704314, 85.72678285, 66.71128591])
to_goal_cost_gain=1
robot_radius=1

#%%
l_dij=list()
l_dw =list()
l_star  =list()
goal_gain=list()
for i in range(6):
    if i == 4:
        to_goal_cost_gain=10
    if i == 0:
        to_goal_cost_gain=0
    else:
        to_goal_cost_gain=1*(i+1)
    
    print(to_goal_cost_gain)
    #DIJKSTRA APPROACH
    rx_dij,ry_dij=dijkstra_approach.main(ob,Ps,to_goal_cost_gain)
    l_dij.append([rx_dij,ry_dij])
    
    #DYNAMIC WINDOW APPROACH
    traj = Dynamic_Window.main(ob,Ps,to_goal_cost_gain)
    rx_dw = traj[:,0]
    ry_dw = traj[:,1]
    l_dw.append([rx_dw,ry_dw])
    
    #A*
    final_path=pathv1.main(to_goal_cost_gain,robot_radius,ob,Ps)
    Astar=np.array(final_path)
    rx_star = Astar[:,0]
    ry_star = Astar[:,1]
    l_star.append([rx_star,ry_star]) 

    
    goal_gain.append(to_goal_cost_gain)
    

#%% plot simulation for all

fig1, ax1 = plt.subplots()
fig2, ax2 = plt.subplots()
fig3, ax3 = plt.subplots()
ax1.cla()
ax2.cla()
ax3.cla()
Len_dij=np.zeros(len(l_dij))
Len_dw =np.zeros(len(l_dw))
Len_star =np.zeros(len(l_star))
'''plot routes'''
for i in range(len(l_dij)):
    '''compute all the lengths'''
    #DIJKSTRA
    dat_dij=l_dij[:][:][i]
    rx_dij,ry_dij=dat_dij
    Len_dij[i]=pathlength(rx_dij,ry_dij)
    #DYNAMIC WINDOW
    dat_dw =l_dw[:][:][i]
    rx_dw,ry_dw=dat_dw
    Len_dw[i]=pathlength(rx_dw,ry_dw)
    #A*
    dat_star =l_star[:][:][i]
    rx_star,ry_star=dat_star
    Len_star[i]=pathlength(rx_star,ry_star)
    
    '''plot trajectory'''
    ax1.plot(rx_dij,ry_dij,'-',label='gain: {d}'.format(d=goal_gain[i]))
    ax2.plot(rx_dw,ry_dw,'-',label='gain: {d}'.format(d=goal_gain[i]))   
    ax3.plot(rx_star,ry_star,'-',label='gain: {d}'.format(d=goal_gain[i]))  
ax1.plot(0,0,'xr')
ax1.plot(14,14,'xb')
ax1.legend()
ax1.grid(True,which='both')
ax1.set_title('Dijkstra, Varying Goal to Cost Gain')

ax2.plot(0,0,'xr')
ax2.plot(14,14,'xb')
ax2.legend()
ax2.grid(True,which='both')
ax2.set_title('Dynamic Window Approach, Varying Goal to Cost Gain')

ax3.plot(0,0,'xr')
ax3.plot(14,14,'xb')
ax3.legend()
ax3.grid(True,which='both')
ax3.set_title('A*, Varying Goal to Cost Gain')
#ax1.legend()
ob_a=np.array(ob)
for i in range(len(ob)):
    xb = ob_a[i,0]
    yb = ob_a[i,1]
    ax1.plot(xb, yb, 'bo',markersize=7)
    ax1.text(xb * (1 + 0.01), yb * (1 + 0.01),np.round(Ps_p[i],2), fontsize=10.5)
    ax2.plot(xb, yb, 'bo',markersize=7)
    ax2.text(xb * (1 + 0.01), yb * (1 + 0.01),np.round(Ps_p[i],2), fontsize=10.5)
    ax3.plot(xb, yb, 'bo',markersize=7)
    ax3.text(xb * (1 + 0.01), yb * (1 + 0.01),np.round(Ps_p[i],2), fontsize=10.5)

    
 #%%   
#plt.hist([Len_dij,Len_dw,Len_star])
##finalPath=pathv1.main(to_goal_cost_gain,robot_radius)
#%%Astar=np.array(final_path)
#
x=Astar[:,0]
y=Astar[:,1]
#L_star=pathlength(x,y)
#print(L_star)
plt.plot(x,y,"-r")
plt.plot(0, 0, "xr")
plt.plot(14, 14, "xb")
plt.grid(True,which='both')
ob_a=np.array(ob)
for i in range(len(ob)):
    xb = ob_a[i,0]
    yb = ob_a[i,1]
    plt.plot(xb, yb, 'bo',markersize=9)
    plt.text(xb * (1 + 0.01), yb * (1 + 0.01),np.round(Ps[i],2), fontsize=10)
#%% saving data
np.save('l_dij',l_dij)
np.save('l_dw',l_dw)
np.save('l_star',l_star)    
##plt.plot(traj[:, 0], traj[:, 1], "-r")
#%% plot cells

def checkerboard_table(df,l_dij):
    '''plot map'''
    fig, ax = plt.subplots()#initializing plot
    ax.cla()
    ax.set_axis_off()
    tb = Table(ax, bbox=[0,0,1,1])

    nrows, ncols = df.shape
    
    vals = np.around(df.values,2)
    normal = matplotlib.colors.Normalize(vals.min()-1, vals.max()+1)
    the_table=ax.table(cellText=vals,
                        colWidths = [0.06]*vals.shape[1], loc='center', 
                        cellColours=plt.cm.cool(normal(vals)))
    the_table.scale(1.006,1.093)
    for i in range(len(l_dij)):
    #DIJKSTRA
        dat_dij=l_dij[:][:][i]
        x,y=dat_dij
        
        
        ax.plot(x,y,'-',label='gain: {d}'.format(d=i))
    ax.plot(0,0,'xr')
    ax.plot(14,14,'xb')
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.18),
          ncol=3, fancybox=True)
    #ax.set_title('Dijkstra, varying cost of gain')

data=np.zeros([16,16])
ncols=16
for i in range(len(ob)):
    #print(i)
    ygrid=(int(ob[i,0]))
    xgrid=(15-int(ob[i,1]))
    #print(xgrid,ygrid)
    data[xgrid,ygrid]=Ps[i]
df = pandas.DataFrame(data, 
            columns=np.arange(0,ncols))


checkerboard_table(df,l_star) #plotting map data on grid
plt.show()
#%%
#df_reward=reward_table(df) #determine reward grid
#checkerboard_table(df_reward) #plot reward grid
#plt.show()
    
    
    


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

#ob = np.matrix([[0, 2],
#                [2.0, 8.0],
#                [2.0, 2.0],
#                [4.0, 2.0],
#                [5.0, 4.0],
#                [5.0, 5.0],
#                [5.0, 6.0],
#                [5.0, 9.0],
#                [8.0, 8.0],
#                [8.0, 9.0],
#                [7.0, 9.0],
#                [7.0, 6.2],
#                [8.0, 5.0],
#                [8.0, 12.0],
#                [9.0, 8.0],
#                [9.0, 2.0],
#                [10.0, 9.0],
#                [11.0, 9.0],
#                [12.0, 6.0],
#                [10.0, 10.0],
#                [12.0, 12.0],
#                [12.0, 14.0],
#                [13.0, 13.0]
#                ])
#    
#    Ps_p=([0.18822291, 0.05969734, 0.74069164, 0.47933208, 0.48850482,
#       0.02146035, 0.31580583, 0.3587886 , 0.79536192, 0.97236535,
#       0.15689085, 0.10305778, 0.39755615, 0.67396048, 0.89009032,
#       0.10704731, 0.70899104, 0.13591165, 0.79673235, 0.27865084,
#       0.29617043, 0.85726783, 0.66711286])
#    
#Ps=([18.82229137,  5.96973403, 74.06916379, 47.93320819, 48.85048176,
#        2.14603484, 31.5805826 , 35.87885955, 79.53619248, 97.23653457,
#       15.68908499, 10.30577757, 39.7556145 , 67.39604789, 89.00903243,
#       10.70473092, 70.89910441, 13.59116515, 79.67323489, 27.86508364,
#       29.61704314, 85.72678285, 66.71128591])


'''ob = np.matrix([[0, 2],
                [2.0, 8.0],
                [2.0, 2.0],
                [4.0, 2.0],
                [5.0, 4.0],
                [5.0, 5.0],
                [5.0, 6.0],
                [5.0, 9.0],
                [6.0, 9.0],
                [8.0, 3.0],
                [9.0, 3.0],
                [8.0, 8.0],
                [8.0, 9.0],
                [7.0, 9.0],
                [7.0, 6.0],
                [7.0, 10.0],
                [8.0, 5.0],
                [8.0, 2.0],
                [8.0, 12.0],
                [9.0, 8.0],
                [9.0, 2.0],
                [10.0, 9.0],
                [11.0, 9.0],
                [12.0, 6.0],
                [10.0, 10.0],
                [12.0, 12.0],
                [12.0, 14.0],
                [13.0, 13.0]
                ])
Ps_p=([0.49139684, 0.39048272, 0.4549528 , 0.34102715, 0.20194511,
       0.23613699, 0.23850729, 0.44884213, 0.29081518, 0.17696865,
       0.97731589, 0.1804784 , 0.2764076 , 0.87958652, 0.10316262,
       0.7168419 , 0.59021069, 0.37572083, 0.42225505, 0.54058026,
       0.20336709, 0.91829905, 0.01759289, 0.44647942, 0.19817146,
       0.47457293, 0.35954698, 0.04275995])
Ps=Ps_p*100  '''
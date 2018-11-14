#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 12 12:08:12 2018
Creating MDP environement 
@author: cngo
"""

import matplotlib.pyplot as plt
from matplotlib.table import Table
import matplotlib
import numpy as np
import numpy as np
import pandas

# %matplotlib auto or inline

def main():
    #generating map data 
    r_len = 5 #size of obstacle
    nrows = 23
    ncols = 19
    data = np.zeros([nrows,ncols])
    data[0:5,0:5] = np.random.rand(5, 5)
    data[18:18+r_len,10:10+r_len] = np.random.rand(r_len,r_len)
    data[7:7+r_len+1,12:12+r_len+1] = np.random.rand(r_len+1,r_len+1)
    data[9:9+r_len-1,4:4+r_len-1] = np.random.rand(r_len-1,r_len-1)
    
    df = pandas.DataFrame(data, 
                columns=np.arange(0,ncols))
    
    checkerboard_table(df) #plotting map data on grid
    plt.show()
    
    df_reward=reward_table(df) #determine reward grid
    checkerboard_table(df_reward) #plot reward grid
    plt.show()
    
    
    

def checkerboard_table(df):
    '''plot map'''
    fig, ax = plt.subplots()#initializing plot
    ax.set_axis_off()
    tb = Table(ax, bbox=[0,0,1,1])

    nrows, ncols = df.shape
    
    vals = np.around(df.values,2)
    normal = matplotlib.colors.Normalize(vals.min()-1, vals.max()+1)
    the_table=plt.table(cellText=vals, rowLabels=df.index, colLabels=df.columns, 
                        colWidths = [0.07]*vals.shape[1], loc='center', 
                        cellColours=plt.cm.cool(normal(vals)))

    return the_table

def reward_table(df):
    '''Create reward table
    intercept=0'''
    slope = -10
    df_reward=df*slope
    return df_reward

if __name__ == '__main__':
    main()


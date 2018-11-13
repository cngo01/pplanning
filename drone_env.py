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

def checkerboard_table(df, fmt='{:.2f}'):
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

if __name__ == '__main__':
    main()


## create discrete colormap
#cmap = my_cmap2#colors.ListedColormap(['red', 'blue'])
#
#fig, ax = plt.subplots()
#ax.imshow(data, cmap=cmap)
#
## draw gridlines
#ax.grid(which='major', axis='both', linestyle='-', color='k', linewidth=2)
#ax.set_xticks(np.arange(0, ncols+1, 1));
#ax.set_yticks(np.arange(0, nrows+1, 1));
#
#plt.show()
#
#
#
##
##def get_color():
##    fact = 1.0/255.0
##    cdict2 = {'red':  [(0.0,   22*fact,  22*fact),
##                       (0.25, 133*fact, 133*fact),
##                       (0.5,  191*fact, 191*fact),
##                       (0.75, 151*fact, 151*fact),
##                       (1.0,   25*fact,  25*fact)],
##             'green': [(0.0,   65*fact,  65*fact),
##                       (0.25, 182*fact, 182*fact),
##                       (0.5,  217*fact, 217*fact),
##                       (0.75, 203*fact, 203*fact),
##                       (1.0,   88*fact,  88*fact)],
##             'blue':  [(0.0,  153*fact, 153*fact),
##                       (0.25, 222*fact, 222*fact),
##                       (0.5,  214*fact, 214*fact),
##                       (0.75, 143*fact, 143*fact),
##                       (1.0,   40*fact,  40*fact)]} 
##    my_cmap2 = matplotlib.colors.LinearSegmentedColormap('my_colormap2',cdict2,256)
##    return my_cmap2
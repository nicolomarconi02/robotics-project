#!/usr/bin/python3

import sys

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D

import numpy
from numpy.random import randn
from scipy import array, newaxis


from mpl_toolkits import mplot3d 
import numpy as np 
import matplotlib.pyplot as plt 

def plot_graph(data):

    Xs = data[:,0]
    Ys = data[:,1]
    Zs = data[:,2]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    surf = ax.plot_trisurf(Ys, Xs, Zs, cmap=cm.jet, linewidth=0)
    fig.colorbar(surf)

    ax.xaxis.set_major_locator(MaxNLocator(5))
    ax.yaxis.set_major_locator(MaxNLocator(6))
    ax.zaxis.set_major_locator(MaxNLocator(5))

    fig.tight_layout()

    ax.invert_yaxis()
    ax.invert_xaxis()
    ax.invert_zaxis()

    ax.set_xlabel("y")
    ax.set_ylabel("x")
    ax.set_zlabel("z")

    ax.view_init(-152, -94) 

    fig.savefig('src/vision/3D.png')
    plt.show()

def plot_graph_v2(data):
    
    fig = plt.figure(figsize = (8, 8)) 
    ax = plt.axes(projection = '3d') 
    
    # Data for a three-dimensional line 
    z = data[:,2]  
    x = data[:,0] 
    y = data[:,1] 
    ax.plot3D(x, y, z, 'green') 
    
    ax.view_init(120, 30) 
    
    plt.show()

def plot_graph_2d(data):
    x = data[:,0]
    y = data[:,1]

    plt.plot(y,x)
    plt.xlabel("y")
    plt.ylabel("x")
    plt.show()

# -*- coding: utf-8 -*-
"""
Created on Sat Nov 30 17:23:05 2024

@author: vande
"""

#Dikke test
from gurobipy import *
import numpy as np
import math
import copy
import pandas as pd
import matplotlib.pyplot as plt


#Read data file
with open("Data/data_small.txt", "r") as f:          # Open Li & Lim PDPTW instance definitions
    data = f.readlines()                        # Extract instance definitions

VRP = []                                        # Create array for data related to nodes
i=0                                             # Varible to keep track of lines in data file
for line in data:
    i=i+1
    words = line.split()
    words=[int(i) for i in words]           # Covert data from string to integer
    VRP.append(words)                       # Store node data
VRP = np.array(VRP)

#print(VRP)

xc=VRP[:,1]                                     # X-position of nodes
yc=VRP[:,2]                                     # Y-position of nodes

nodes = VRP[:, 0]
n = len(nodes)

s=np.zeros((n,n))                               # Create array for distance between nodes
for i in nodes:
    for j in nodes:
        s[i][j]=math.sqrt((xc[j] - xc[i])**2 + (yc[j] - yc[i])**2) # Store distance between nodes
        
print(s)




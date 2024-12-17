from gurobipy import *
import numpy as np
import math
import copy
import pandas as pd
import matplotlib.pyplot as plt

with open("Data/data_large_multiTW_diff.txt", "r") as f:          # Open Li & Lim PDPTW instance definitions
    data = f.readlines()        

VRP = []                                        # Create array for data related to nodes
max_length = max(len(line.split()) for line in data)  # Find the longest row
i = 0                                             # Varible to keep track of lines in data file
for line in data:
    print(line)
    i = i + 1
    words = line.split()
    words = [int(i) for i in words]           # Covert data from string to integer
    while len(words) < max_length:
        words.append(np.nan)  # Pad with NaN (or another placeholder)
    VRP.append(words)                       # Store node data
VRP = np.array(VRP)

print(VRP)
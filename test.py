# -*- coding: utf-8 -*-
"""
Created on Sat Nov 30 17:23:05 2024

@author: vande
"""

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

print(VRP)

xc=VRP[:,1]                                     # X-position of nodes
yc=VRP[:,2]                                     # Y-position of nodes

nodes = VRP[:, 0]
N = VRP[:, 0]
n = len(nodes)

s=np.zeros((n,n))                               # Create array for distance between nodes
for i in nodes:
    for j in nodes:
        s[i][j]=math.sqrt((xc[j] - xc[i])**2 + (yc[j] - yc[i])**2) # Store distance between nodes
        

V = range(2)

m = Model('VRPmodel')

## Decision variables

# binary variable, 1 if vehicle v travels from i to j, 0 if not
b = {}
for i in N:
    for j in N:
        for v in V:
            b[i, j, v] = m.addVar(vtype=GRB.BINARY, lb = 0)
            
#binary variable, 1 if vehicle v visits location j, 0 if not
z = {}
for j in N:
    for v in V:
        z[j, v] = m.addVar(vtype=GRB.BINARY, lb=0)
        
# arrival time of vehicle v at location i
t = {}
for i in N:
    for v in V:
        t[i, v] = m.addVar(vtype=GRB.CONTINUOUS, lb=0)
        
# Add position variables for subtour elimination
u = {}
for i in N:
    u[i] = m.addVar(lb=0, ub=n, vtype=GRB.CONTINUOUS, name=f"u[{i}]")  # Position of node i in the route

        
##Objective
obj = (quicksum(s[i, j] * b[i,j,v] for i in N for j in N if i != j for v in V))
m.setObjective(obj, GRB.MINIMIZE)

##Constraints


#Every location visited exactly once        
for j in N:
    if j != 0:
        m.addConstr(quicksum(z[j,v] for v in V) == 1)
    
#Flow continuity
for j in N:
    for v in V:
        m.addConstr((quicksum(b[i,j,v] for i in N if i != j)) == (quicksum(b[j,i,v] for i in N if i != j)))

#start at depot
for v in V:
    m.addConstr(quicksum(b[0,j,v] for j in N[1:]) == 1)
    m.addConstr(quicksum(b[i,0,v] for i in N[1:]) == 1)
    
for v in V:
    for j in N:
        if j != 0:
            m.addConstr(quicksum(b[i, j, v] for i in N if i != j) == z[j, v])
    
# Add subtour elimination constraints
for i in N:
    for j in N:
        if i != j and j != 0:  # Exclude depot as it does not need ordering
            for v in V:
                m.addConstr(u[i] + 1 - n * (1 - b[i, j, v]) <= u[j],
                            name=f"SubtourElim[{i},{j},{v}]")




m.update()

m.optimize()

print(m.objVal)
#print(z)

print("Nodes:", N)
print("Decision Variables (z):", z)

if m.status == GRB.OPTIMAL:
    print(f"Optimal objective value (total distance): {m.objVal}")
    
    # Extract routes for each vehicle
    routes = {v: [] for v in V}  # Dictionary to store the route for each vehicle
    for v in V:
        current_node = 0  # Start at the depot
        route = [current_node]  # Initialize route with the depot
        
        while True:
            # Find the next node connected to the current node for this vehicle
            next_node = None
            for j in N:
                if current_node != j and b[current_node, j, v].X > 0.5:  # Decision variable > 0.5 indicates selection
                    next_node = j
                    break
            
            if next_node is None or next_node == 0:  # Return to depot or no more nodes to visit
                route.append(0)  # Append depot at the end
                break
            
            route.append(next_node)
            current_node = next_node
        
        routes[v] = route  # Save the route for this vehicle
    
    # Print routes
    for v, route in routes.items():
        print(f"Vehicle {v}: {' -> '.join(map(str, route))}")
else:
    print("No optimal solution found.")
    
arc_solution = m.getAttr('x', b)

# Plot the routes
fig = plt.figure(figsize=(10, 10))
plt.xlabel('x-coordinate')
plt.ylabel('y-coordinate')
plt.title('Vehicle Routing Problem Solution')

# Plot all nodes
plt.scatter(xc, yc, c='blue', label='Nodes')
for i in range(n):
    plt.annotate(str(i), (xc[i], yc[i]), textcoords="offset points", xytext=(5, 5), ha='center')

# Mark the depot
plt.scatter(xc[0], yc[0], c='green', marker='s', s=100, label='Depot')

# Plot the routes for each vehicle
colors = ['red', 'orange', 'purple', 'brown', 'cyan']  # Use distinct colors for vehicles
for v in V:
    for i in N:
        for j in N:
            if arc_solution[i, j, v] > 0.99:  # Check if route is selected
                plt.plot([xc[i], xc[j]], [yc[i], yc[j]], linestyle='--', color=colors[v % len(colors)], label=f'Vehicle {v}' if i == 0 else "")

plt.legend()
plt.show()










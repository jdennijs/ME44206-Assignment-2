# -*- coding: utf-8 -*-
#"""
#Created on Sat Nov 30 17:23:05 2024
#
#@author: Ivo Aben, Jelle Derks, Jochem den Nijs & Wouter van der Hoorn
#"""

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
i = 0                                             # Varible to keep track of lines in data file
for line in data:
    i = i + 1
    words = line.split()
    words = [int(i) for i in words]           # Covert data from string to integer
    VRP.append(words)                       # Store node data
VRP = np.array(VRP)

print(VRP)

xc = VRP[:,1]                                     # X-position of nodes
yc = VRP[:,2]                                     # Y-position of nodes

nodes = VRP[:, 0]
N = VRP[:, 0]
n = len(nodes)

s = np.zeros((n,n))                               # Create array for distance between nodes
for i in nodes:
    for j in nodes:
        s[i][j]=math.sqrt((xc[j] - xc[i])**2 + (yc[j] - yc[i])**2) # Store distance between nodes
        
V = range(2)

C = 130 #capacity of each vehicle

d = VRP[:,3] #Demand at a stop


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
    u[i] = m.addVar(lb=0, ub=n, vtype=GRB.CONTINUOUS, name=f"u[{i}]")  # Position (order) of node i in the route

        
## Objective
obj = (quicksum(s[i, j] * b[i,j,v] for i in N for j in N if i != j for v in V))
m.setObjective(obj, GRB.MINIMIZE)

##Constraints


#Constraint 1
#Every location visited exactly once  
      
for j in N:
    if j != 0:
        m.addConstr(quicksum(z[j,v] for v in V) == 1)
    
#Constraint 2
#Flow continuity
for j in N:
    for v in V:
        m.addConstr((quicksum(b[i,j,v] for i in N if i != j)) == (quicksum(b[j,i,v] for i in N if i != j)))

#Constraint 3
#start at depot
for v in V:
    m.addConstr(quicksum(b[0,j,v] for j in N[1:]) == 1)
    
#Constraint  4
#end at depot
for v in V:
    m.addConstr(quicksum(b[i,0,v] for i in N[1:]) == 1)
    
#Constraint 5
for v in V:
    for j in N:
        if j != 0:
            m.addConstr(quicksum(b[i, j, v] for i in N if i != j) == z[j, v])
    

#Constraint 6
# Add subtour elimination constraints
for i in N:
    for j in N:
        if i != j and j != 0:  # Exclude depot as it does not need ordering
            for v in V:
                m.addConstr(u[i] + 1 - n * (1 - b[i, j, v]) <= u[j])

#Constraint 7
#Vehicle capacity constraint
for v in V:
    m.addConstr(quicksum(d[j] * z[j, v] for j in N) <= C)


m.update()

m.optimize()

print(m.objVal)
#print(z)

print("Nodes:", N)
print("Decision Variables (z):", z)

   
    
if m.status == GRB.OPTIMAL:
    print(f"Optimal objective value (total distance): {m.objVal}")
    
    # Extract routes and loads for each vehicle
    routes = {v: [] for v in V}  # Dictionary to store the route for each vehicle
    vehicle_loads = {v: 0 for v in V}  # Dictionary to store the load for each vehicle

    for v in V:
        current_node = 0  # Start at the depot
        route = [current_node]  # Initialize route with the depot
        load = 0  # Initialize vehicle load
        
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
            load += d[next_node]  # Add the demand of the visited node to the load
            current_node = next_node
        
        routes[v] = route  # Save the route for this vehicle
        vehicle_loads[v] = load  # Save the load for this vehicle

    # Print routes and loads
    for v, route in routes.items():
        print(f"Vehicle {v}: {' -> '.join(map(str, route))}")
        print(f"Vehicle {v} carries a total load of: {vehicle_loads[v]}")

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










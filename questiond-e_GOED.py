# -*- coding: utf-8 -*-
"""
Created on Sat Nov 30 17:23:05 2024

@author: Ivo Aben, Jelle Derks, Jochem den Nijs and Wouter van der Hoorn
"""

from gurobipy import *
import numpy as np
import math
import copy
import pandas as pd
import matplotlib.pyplot as plt

#Read data file
with open("data_small_multiTW.txt", "r") as f:          # Open Li & Lim PDPTW instance definitions
    data = f.readlines()                        # Extract instance definitions

VRP = []                                        # Create array for data related to nodes
i = 0                                             # Varible to keep track of lines in data file
for line in data:
    i = i + 1
    words = line.split()
    words=[int(i) for i in words]           # Covert data from string to integer
    VRP.append(words)                       # Store node data
VRP = np.array(VRP)

xc = VRP[:,1]                                     # X-position of nodes
yc = VRP[:,2]                                     # Y-position of nodes

nodes = VRP[:, 0]
N = VRP[:, 0]
n = len(nodes)

s = np.zeros((n,n))                               # Create array for distance between nodes
for i in nodes:
    for j in nodes:
        s[i][j] = math.sqrt((xc[j] - xc[i])**2 + (yc[j] - yc[i])**2) # Store distance between nodes
        

V = range(2) #Number of vehicles

C = 130 #capacity of each vehicle

d = VRP[:,3] #Demand at a stop

ST = VRP[:,4] #Service time 

w = VRP[:,5] #Number of time windows per stop

RT = []
DT = []

for j, k in enumerate(w):
    RT.append(VRP[j, 6:6 + 2 * k:2].tolist())
    DT.append(VRP[j, 7:7 + 2 * k:2].tolist())

K = []

for j in w:
    K.append(list(range(j)))
    
    
m = Model('VRPmodel')

## Decision variables

# binary variable, 1 if vehicle v travels from i to j, 0 if not
b = {}
for i in N:
    for j in N:
        for v in V:
            b[i, j, v] = m.addVar(vtype=GRB.BINARY, lb = 0)
            
#binary variable, 1 if vehicle v visits location j during time slot k, 0 if not
z = {}
for j in N:
    for k in K[j]:
        for v in V:
            z[j, k, v] = m.addVar(vtype=GRB.BINARY, lb=0)
        
# arrival time of vehicle v at location i
t = {}
for v in V:
    for j in N:
        t[j, v] = m.addVar(vtype=GRB.CONTINUOUS, lb=0)
    

        
##Objective
obj = (quicksum(s[i, j] * b[i,j,v] for i in N for j in N if i != j for v in V))
m.setObjective(obj, GRB.MINIMIZE)

##Constraints
#Constraint 1
#Every location visited exactly once during one time slot
for j in N:
    if j != 0:
        m.addConstr(quicksum(z[j, k, v] for k in K[j] for v in V) == 1)
    
#Constraint 2
#Flow continuity
for j in N:
    for v in V:
        m.addConstr((quicksum(b[i,j,v] for i in N if i != j)) == (quicksum(b[j,i,v] for i in N if i != j)))
    
#Constraint 3
#Link travel route to time window            
for v in V:
    for j in N:
        if j != 0:
            m.addConstr(
                quicksum(b[i, j, v] for i in N if i != j) == quicksum(z[j, k, v] for k in K[j]))


#Constraint 4
#Vehicle capacity constraint
for v in V:
    m.addConstr(
        quicksum(d[j] * quicksum(z[j, k, v] for k in K[j]) for j in N) <= C)
    
#Constraint 5
M = 1e5 + 1e6
for v in V:
    for i in N:
        for j in N:
            if i != j and j != 0:  # Ensure it's not the depot
                m.addConstr(t[j, v] >= t[i, v] + s[i, j] + ST[i] - M * (1 - b[i, j, v]))
#Constraint 6
#Vehicle arrives at stop before due time
for v in V:
    for j in N:
        if j != 0:
            for k in K[j]:
                m.addConstr(t[j, v] <= DT[j][k] + M * (1 - z[j, k, v]))

# #Constraint 7:
# #Vehicle arrives at stop after ready time
for v in V:
    for j in N:
        for k in K[j]:    
            m.addConstr(t[j, v] >= RT[j][k] - M * (1 - z[j, k, v]))

# Constraint 8: abundance; have to have been at 0 at some point
for v in V:
    for j in N:
        m.addConstr(quicksum(z[0, k, v] for k in K[j]) == 1)  

m.update()

m.optimize()


if m.status == GRB.OPTIMAL:
    print(f"Optimal objective value (total distance): {m.objVal:.2f}")
    
    # Extract routes, loads, and arrival times for each vehicle
    routes = {v: [] for v in V}  # Dictionary to store the route for each vehicle
    vehicle_loads = {v: 0 for v in V}  # Dictionary to store the load for each vehicle
    vehicle_times = {v: [] for v in V}  # Dictionary to store arrival times for each vehicle
    vehicle_time_slots = {v: [] for v in V}  # Dictionary to store time slots for each vehicle

    for v in V:
        current_node = 0  # Start at the depot
        route = [current_node]  # Initialize route with the depot
        load = 0  # Initialize vehicle load
        times = [t[current_node, v].X]  # Start with the depot's time (should be 0)
        time_slots = [None]  # Depot has no time window

        while True:
            # Find the next node connected to the current node for this vehicle
            next_node = None
            for j in N:
                if current_node != j and b[current_node, j, v].X > 0.5:  # Decision variable > 0.5 indicates selection
                    next_node = j
                    break
            
            if next_node is None or next_node == 0:  # Return to depot or no more nodes to visit
                last_node = route[-1]
                route.append(0)  # Append depot at the end
                times.append(t[last_node, v].X + s[last_node, 0] + ST[last_node])  # Append depot's return time
                time_slots.append(None)  # Depot has no time window
                break
            
            route.append(next_node)
            times.append(t[next_node, v].X)  # Record the arrival time at the next node
            load += d[next_node]  # Add the demand of the visited node to the load
            
            # Identify the time slot used at the next node
            used_slot = None
            for k in K[next_node]:
                if z[next_node, k, v].X > 0.5:  # Decision variable > 0.5 indicates selection
                    used_slot = k
                    break
            time_slots.append(used_slot)
            
            current_node = next_node
        
        routes[v] = route  # Save the route for this vehicle
        vehicle_loads[v] = load  # Save the load for this vehicle
        vehicle_times[v] = times  # Save the arrival times for this vehicle
        vehicle_time_slots[v] = time_slots  # Save the time slots for this vehicle

    # Print routes, loads, arrival times, and time slots
    for v, route in routes.items():
        print(f"Vehicle {v}: {' -> '.join(map(str, route))}")
        print(f"Vehicle {v} carries a total load of: {vehicle_loads[v]}")
        print(f"Vehicle {v} arrival times: {', '.join(f'{time:.2f}' for time in vehicle_times[v])}")
        print(f"Vehicle {v} time slots: {', '.join(str(slot) if slot is not None else 'Depot' for slot in vehicle_time_slots[v])}")

else:
    print("No optimal solution found.")

    
arc_solution = m.getAttr('x', b)

# Plot the routes
fig = plt.figure(dpi= 120, figsize=(10, 10))
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
                plt.plot([xc[i], xc[j]], [yc[i], yc[j]], linestyle='--', color=colors[v % len(colors)], label=f'Vehicle {v}' if v == j else "")

plt.legend()
plt.savefig('Figs/questiond-e.png')
plt.show()









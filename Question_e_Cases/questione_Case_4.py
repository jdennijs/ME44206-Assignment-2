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
with open("data_small.txt", "r") as f:          
    data = f.readlines()                        

VRP = []                                        # Create array for data related to nodes
i = 0                                             # Varible to keep track of lines in data file
for line in data:
    i = i + 1
    words = line.split()
    words=[int(i) for i in words]           # Covert data from string to integer
    VRP.append(words)                       # Store node data
VRP = np.array(VRP)

xc = VRP[:,1]                          # X-position of nodes
yc = VRP[:,2]                          # Y-position of nodes

nodes = VRP[:, 0]
N = VRP[:, 0]
n = len(nodes)              #Number of nodes

s = np.zeros((n,n))         # Create array for distance between nodes
for i in nodes:
    for j in nodes:
        s[i][j] = math.sqrt((xc[j] - xc[i])**2 + (yc[j] - yc[i])**2) # Store distance between nodes
        

V = range(3) #Number of vehicles

C = 70 #capacity of each vehicle

d = VRP[:,3] #Demand at a stop

ST = VRP[:,4] #Service time 

RT = VRP[:,5] #Ready time

DT = VRP[:,6] #Due time


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
for v in V:
    for j in N:
        t[j, v] = m.addVar(vtype=GRB.CONTINUOUS, lb=0)

        
##Objective
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
for v in V:
    for j in N:
        if j != 0:
            m.addConstr(quicksum(b[i, j, v] for i in N if i != j) == z[j, v])

#Constraint 4
#Vehicle capacity constraint
for v in V:
    m.addConstr(quicksum(d[j] * z[j, v] for j in N) <= C)
    
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
            m.addConstr(t[j, v] <= DT[j])

# #Constraint 7:
# #Vehicle arrives at stop after ready time
for v in V:
    for j in N:
        m.addConstr(t[j, v] >= RT[j])
        

m.update()

m.optimize()


if m.status == GRB.OPTIMAL:
    print(f"Optimal objective value (total distance): {m.objVal:.2f}")
    
    # Extract routes, loads, and arrival times for each vehicle
    routes = {v: [] for v in V}  # Dictionary to store the route for each vehicle
    vehicle_loads = {v: 0 for v in V}  # Dictionary to store the load for each vehicle
    vehicle_times = {v: [] for v in V}  # Dictionary to store arrival times for each vehicle
    order_numbers = {v: [] for v in V}
    load_vehicle = {v: [] for v in V}
    vehicle_loads_sum = {v: [] for v in V}

    for v in V:
        current_node = 0  # Start at the depot
        route = [current_node]  # Initialize route with the depot
        load_start = 0  # Initialize vehicle load
        load = [load_start]
        load_sum = 0
        times = [t[current_node, v].X]  # Start with the depot's time (should be 0)
        order_n = 0 #start routes a depot
        order = [order_n]
        
        while True:
            # Find the next node connected to the current node for this vehicle
            next_node = None
            for j in N:
                if current_node != j and b[current_node, j, v].X > 0.5:  # Decision variable > 0.5 indicates selection
                    next_node = j
                    #order.append(u[j].x)
                    break
            
            if next_node is None or next_node == 0:  # Return to depot or no more nodes to visit
                last_node = route[-1]
                route.append(0)  # Append depot at the end
                times.append(t[last_node, v].X + s[last_node,0] + ST[last_node])  # Append depot's return time
                break
            
            route.append(next_node)
            times.append(t[next_node, v].X)  # Record the arrival time at the next node
            load.append(d[next_node])  # Add the demand of the visited node to the load
            load_sum += d[next_node]  # Add the demand of the visited node to the load
            current_node = next_node
        
        routes[v] = route  # Save the route for this vehicle
        vehicle_loads_sum[v] = load_sum  # Save the load for this vehicle
        vehicle_times[v] = times  # Save the arrival times for this vehicle
        order_numbers[v] = order
        load_vehicle[v] = load

    # Print routes, loads, and arrival times
    for v, route in routes.items():
        print(f"Vehicle {v}: {' -> '.join(map(str, route))}")
        print(f"Vehicle {v} carries a total load of: {load_sum}")
        print(f"Vehicle {v} loads: {', '.join(f'{i:.2f}' for i in load_vehicle[v])}")
        print(f"Vehicle {v} arrival times: {', '.join(f'{time:.2f}' for time in vehicle_times[v])}")

else:
    print("No optimal solution found.")
    
arc_solution = m.getAttr('x', b)

# Plot the routes
fig = plt.figure(dpi= 120, figsize=(7, 7))
plt.xlabel('x-coordinate')
plt.ylabel('y-coordinate')
plt.title('Vehicle Routing Problem Solution Case 4')

# Plot all nodes
plt.scatter(xc, yc, c='blue', label='Nodes')
for i in range(n):
    plt.annotate(str(i), (xc[i], yc[i]), textcoords="offset points", xytext=(5, 5), ha='center')

# Mark the depot
plt.scatter(xc[0], yc[0], c='green', marker='s', s=100, label='Depot')


# Plot the routes for each vehicle
colors = ['red', 'orange', 'purple', 'brown', 'cyan']  # Use distinct colors for vehicles
for v in V:
    first_route_plotted = False  # To track if the label has been added for this vehicle
    for i in N:
        for j in N:
            if arc_solution[i, j, v] > 0.99:  # Check if route is selected
                # Add label only for the first route of the vehicle
                if not first_route_plotted:
                    plt.plot([xc[i], xc[j]], [yc[i], yc[j]], linestyle='--', color=colors[v % len(colors)], label=f'Vehicle {v}')
                    first_route_plotted = True
                else:
                    plt.plot([xc[i], xc[j]], [yc[i], yc[j]], linestyle='--', color=colors[v % len(colors)])


plt.legend()
plt.savefig('../Figs/question_e_case4.png')
plt.show()









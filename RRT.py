# -*- coding: utf-8 -*-
"""
Created on Tue Jul  4 05:15:45 2023

@author: DELL
"""

import numpy as np 
import matplotlib.pyplot as plt
from matplotlib.pyplot import rcParams
import random
from config_space import *


np.set_printoptions(precision=3, suppress=True)
rcParams['font.family'] = 'sans-serif'
rcParams['font.sans-serif'] = 'Verdana'
plt.rcParams['font.size'] = 22
 
#tree class
class tree_node():
    def __init__(self, locationX, locationY):
        self.locationX = locationX
        self.locationY = locationY
        self.children = []
        self.parent = None
       
        
class RRTAlgorithm():
    def __init__(self, start, goal, numIterations, grid, stepSize):
        self.randomTree = tree_node(start[0], start[1])
        self.goal = tree_node(goal[0], goal[1])
        self.nearest_node = None
        self.iterations = min(numIterations, 400)
        self.grid = grid
        self.stepSize = stepSize
        self.path_distance = 0
        self.nearest_distance = 1000
        self.num_waypoints = 0
        self.waypoints = []
        
    def addChild(self, locationX, locationY):
        if (locationX == self.goal.locationX):
            self.nearest_node.children.append(self.goal)
            self.goal.parent = self.nearest_node
        else:
            newNode = tree_node(locationX, locationY)
            # Append the new node to nearestNode's children
            newNode.parent = self.nearest_node
            self.nearest_node.children.append(newNode)
            
            
    def sampleAPoint(self):
        x = random.randint(1, self.grid.shape[0])
        y = random.randint(1, self.grid.shape[1])
        point = np.array([x, y])
        return point
    
    
    def steerToPoint(self, locationStart, locationEnd):
        offset = self.stepSize*self.unitVector(locationStart, locationEnd)
        point = np.array([locationStart.locationX + offset[0], locationStart.locationY + offset[1]])
        if point[0] >= self.grid.shape[1]:
            point[0] = self.grid.shape[1]-1
        if point[1] >= self.grid.shape[0]:
            point[1] = self.grid.shape[0]-1
        return point
    
    def isInObstacle(self, locationStart, locationEnd):
        u_hat = self.unitVector(locationStart, locationEnd)
        testPoint = np.array([0.0, 0.0])
        for i in range(self.stepSize):
            testPoint[0] = min(self.grid.shape[1]-1, locationStart.locationX + i*u_hat[0])
            testPoint[1] = min(self.grid.shape[0]-1,locationStart.locationY + i*u_hat[1])
            if self.grid[round(testPoint[1]),round(testPoint[0])] == 0:
                return True
        return False

    def unitVector(self, locationStart, locationEnd):
        v = np.array([locationEnd[0] - locationStart.locationX, locationEnd[1] - locationStart.locationY])
        u_hat = v/np.linalg.norm(v)
        return u_hat
    
    #find the nearest node from a given (unconnected) point (Euclidean distance) (TODO--------)
    def findNearest(self, root, point):
        if not root:
            return
        # Calculate the Euclidean distance between root and point
        distance = self.distance(root, point)
        # Check if the distance is lower than or equal to nearestDist
        if distance <= self.nearest_distance:
            # Update nearestNode to root
            self.nearest_node = root
            # Update nearest_distance to the calculated distance
            self.nearest_distance = distance
        
        # Recursively search for the nearest node in the children
        for child in root.children:
            self.findNearest(child, point)

    #find euclidean distance between a node object and an XY point
    def distance(self, node1, point):
        dist = np.sqrt((node1.locationX - point[0]) ** 2 + (node1.locationY - point[1]) ** 2)
        return dist
    
    #check if the goal is within stepsize distance from point, return true if so otherwise false 
    def goalFound(self, point):
        distance = self.distance(self.goal, point)
        if distance <= self.stepSize:
            return True
        else:
            return False

    #reset: set nearestNode to None and nearestDistance to 10000 
    def resetNearestValues(self):
        self.nearest_node = None
        self.nearest_distance = 1000
        
    #trace the path from goal to start  

    def moveViaWaypoints(self):
        # Get the mouse coordinates 
        link1_length = 0.20075  # 5
        link2_length = 0.159  # 4
        x =  self.waypoints[:,0]
        y =  self.waypoints[:,1]

         # Compute the inverse kinematics to find theta1 and theta2
        argument = (x ** 2 + y ** 2 - link1_length ** 2 - link2_length ** 2) / (2 * link1_length * link2_length)
        theta2 = np.arccos(np.clip(argument, -1, 1))
        theta1 = np.arctan2(y, x) - np.arcsin((link2_length * np.sin(theta2)) / np.sqrt(x ** 2 + y ** 2))

         # Update the robot arm and end effector positions
        update_robot(theta1, theta2)    

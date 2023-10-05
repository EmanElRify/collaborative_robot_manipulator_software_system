import numpy as np
from RRT import RRTAlgorithm
import matplotlib.pyplot as plt
from matplotlib.pyplot import rcParams
import random

#load the grid, set start and goal <x, y> positions, number of iterations, step size
grid = np.load('configuration_space.npy')
start = np.array([0, 0])
goal = np.array([0.0, 150])
numIterations = 400
stepSize = 20
link1_length = 0.20075  # 5
link2_length = 0.159  # 4

goalRegion = plt.Circle((goal[0], goal[1]), stepSize, color='b', fill = False)

fig = plt.figure("RRT Algorithm")
plt.imshow(1 - grid, cmap='binary')
plt.plot(start[0],start[1],'ro')
plt.plot(goal[0],goal[1],'bo')
ax = fig.gca()
plt.gca().invert_yaxis()

ax.add_patch(goalRegion)
plt.xlabel('Theta 1 $(deg)$')
plt.ylabel('Theta 2 $(deg)$')

rrt = RRTAlgorithm(start, goal, numIterations, grid, stepSize)
# plt.pause(2)

#iterate
for i in range(rrt.iterations):
    #Reset nearest values, call the resetNearestValues method
    rrt.resetNearestValues()
    print("Iteration: ",i)
    #algorithm begins here-------------------------------------------------------------------------------
    
    #sample a point (use the appropriate method, store the point in variable)- call this variable 'point' to match Line 151
    point = rrt.sampleAPoint()
    print(point)
    #find the nearest node w.r.t to the point (just call the method do not return anything)
    rrt.findNearest(rrt.randomTree, point)
    new = rrt.steerToPoint(rrt.nearest_node, point) #steer to a point, return as 'new'
    #if not in obstacle
    if not rrt.isInObstacle(rrt.nearest_node, new):
        #add new to the nearestnode (addChild), again no need to return just call the method
        rrt.addChild(new[0], new[1])

        plt.pause(0.10)
        plt.plot([rrt.nearest_node.locationX, new[0]], [rrt.nearest_node.locationY, new[1]],'go', linestyle="--")  
        #if goal found (new is within goal region)
        if (rrt.goalFound(new)):
            #append goal to path
            rrt.addChild(rrt.goal.locationX, rrt.goal.locationY)
            # rrt.retraceRRTPath(rrt.goal)


            #retrace

            break



# Initialize an empty path list
path = []

# Traverse from the goal node to the start node
current_node = rrt.goal
while not np.array_equal(current_node, rrt.randomTree):
    # Add the current node's location to the path
    path.append(np.array([current_node.locationX, current_node.locationY]))
    
    # Update the current node to its parent
    current_node = current_node.parent

# Add the start node's location to the path
path.insert(0, np.array([rrt.randomTree.locationX, rrt.randomTree.locationY]))

# Update the number of waypoints
rrt.num_waypoints = len(path)

# Update the waypoints list
rrt.waypoints = path

# Update the path distance
rrt.path_distance = rrt.stepSize * (rrt.num_waypoints - 1)

#Add start to waypoints (DONE)
rrt.waypoints.insert(0,start)

print("Number of waypoints: ", rrt.num_waypoints)
print("Path Distance (m): ", rrt.path_distance)    
print("Waypoints: ", rrt.waypoints)

#plot the waypoints in red (DONE)
for i in range(len(rrt.waypoints)-1):
    plt.plot([rrt.waypoints[i][0], rrt.waypoints[i+1][0]], [rrt.waypoints[i][1], rrt.waypoints[i+1][1]],'ro', linestyle="--")
    plt.pause(0.10)


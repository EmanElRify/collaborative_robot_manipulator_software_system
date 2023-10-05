import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point

def generate_configuration_space(theta1_range, theta2_range, obstacle_positions, obstacle_sizes, resolution=1):
    nrows = len(theta2_range)
    ncols = len(theta1_range)

    cspace = np.ones((nrows, ncols), dtype=bool)

    for i, theta2 in enumerate(theta2_range):
        for j, theta1 in enumerate(theta1_range):
            if collision_check(theta1, theta2, obstacle_positions, obstacle_sizes):
                cspace[i, j] = False

    return cspace

def collision_check(theta1, theta2, obstacle_positions, obstacle_sizes):
    # Define the robot arm links
    link1_length = 0.20075
    link2_length = 0.159

    # Compute the end effector position
    x1 = link1_length * np.cos(theta1)
    y1 = link1_length * np.sin(theta1)
    x2 = x1 + link2_length * np.cos(theta1 + theta2)
    y2 = y1 + link2_length * np.sin(theta1 + theta2)

    # Create the polygons for the arm links
    link1_vertices = np.array([[0, 0], [link1_length, 0], [x1, y1]])
    link2_vertices = np.array([[x1, y1], [link2_length, 0], [x2, y2]])

    link1_polygon = Polygon(link1_vertices)
    link2_polygon = Polygon(link2_vertices)

    # Check for collision with each obstacle
    for obstacle_position, obstacle_size in zip(obstacle_positions, obstacle_sizes):
        obstacle_rectangle = Polygon([
            (obstacle_position[0] - obstacle_size[0] / 2, obstacle_position[1] - obstacle_size[1] / 2),
            (obstacle_position[0] - obstacle_size[0] / 2, obstacle_position[1] + obstacle_size[1] / 2),
            (obstacle_position[0] + obstacle_size[0] / 2, obstacle_position[1] + obstacle_size[1] / 2),
            (obstacle_position[0] + obstacle_size[0] / 2, obstacle_position[1] - obstacle_size[1] / 2)
        ])

        if link1_polygon.intersects(obstacle_rectangle) or link2_polygon.intersects(obstacle_rectangle):
            return True

    return False

# Define the range of theta1 and theta2 in radians
theta1_range = np.arange(-np.pi, np.pi, np.pi/100)
theta2_range = np.arange(-np.pi, np.pi, np.pi/100)

# Define the positions and sizes of the obstacles
obstacle_positions = [(0.2, 0.2), (-0.1, 0.15)]
obstacle_sizes = [(0.1, 0.15), (0.1, 0.1)]

# Generate the configuration space
cspace = generate_configuration_space(theta1_range, theta2_range, obstacle_positions, obstacle_sizes)
np.save('configuration_space.npy', cspace)


# Define the lengths of the robot arm links
link1_length = 0.20075  # 5
link2_length = 0.159  # 4
full_length_arm = link1_length+link2_length
# Define the workspace boundaries
workspace_x_min = -full_length_arm
workspace_x_max = full_length_arm
workspace_y_min = -full_length_arm
workspace_y_max = full_length_arm

# Create subplots for workspace and configuration space
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

# Plot the workspace
ax1.set_xlim(workspace_x_min, workspace_x_max)
ax1.set_ylim(workspace_y_min, workspace_y_max)
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_title('Workspace')

# Initialize the end effector position in the workspace
end_effector_x = link1_length + link2_length
end_effector_y = 0

# Plot the robot arm
link1 = ax1.plot([0, link1_length], [0, 0], 'b', linewidth=2)[0]
link2 = ax1.plot([link1_length, end_effector_x], [0, end_effector_y], 'b', linewidth=2)[0]

# Plot the obstacles
for obstacle_position, obstacle_size in zip(obstacle_positions, obstacle_sizes):
    obstacle_rectangle = plt.Rectangle(
        (obstacle_position[0] - obstacle_size[0] / 2, obstacle_position[1] - obstacle_size[1] / 2),
        obstacle_size[0], obstacle_size[1], color='red'
    )
    ax1.add_artist(obstacle_rectangle)

# Plot the end effector position in the workspace
end_effector = ax1.plot([end_effector_x], [end_effector_y], 'ro', markersize=6)[0]

# Plot the configuration space
ax2.imshow(cspace.astype(int), origin='lower', extent=[-np.pi, np.pi, -np.pi, np.pi], cmap='gray')
ax2.set_xlabel('Theta1 (radians)')
ax2.set_ylabel('Theta2 (radians)')
ax2.set_title('Configuration Space')

# Initialize the end effector position in the configuration space
end_effector_theta1 = 0
end_effector_theta2 = 0

# Plot the end effector position in the configuration space
end_effector_config = ax2.plot([end_effector_theta1], [end_effector_theta2], 'ro', markersize=6)[0]

def update_robot(theta1, theta2):
    # Compute the end effector position
    x = link1_length * np.cos(theta1) + link2_length * np.cos(theta1 + theta2)
    y = link1_length * np.sin(theta1) + link2_length * np.sin(theta1 + theta2)

    # Update the robot arm plot
    link1.set_data([0, link1_length * np.cos(theta1)], [0, link1_length * np.sin(theta1)])
    link2.set_data([link1_length * np.cos(theta1), x], [link1_length * np.sin(theta1), y])

    # Update the end effector position in the workspace
    end_effector.set_data([x], [y])

    # Update the end effector position in the configuration space
    end_effector_config.set_data([theta1], [theta2])

    # Redraw the plot
    plt.draw()

# Function to handle mouse movement
def on_mouse_move(event):
    if event.inaxes == ax1:
        # Get the mouse coordinates
        x = event.xdata
        y = event.ydata

        # Compute the inverse kinematics to find theta1 and theta2
        argument = (x ** 2 + y ** 2 - link1_length ** 2 - link2_length ** 2) / (2 * link1_length * link2_length)
        theta2 = np.arccos(np.clip(argument, -1, 1))
        theta1 = np.arctan2(y, x) - np.arcsin((link2_length * np.sin(theta2)) / np.sqrt(x ** 2 + y ** 2))

        # Update the robot arm and end effector positions
        update_robot(theta1, theta2)

# Connect the mouse movement event to the handler function
plt.connect('motion_notify_event', on_mouse_move)

# Show the plot
plt.show()


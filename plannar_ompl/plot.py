import matplotlib.pyplot as plt
import matplotlib.patches as patches
import csv
import numpy as np

# Read the path from CSV file
all_positions = []

with open('./data/path.csv', 'r') as csvfile:
    csvreader = csv.reader(csvfile)
    for row in csvreader:
        positions = []
        for i in range(0, len(row) - 1, 2):
            x = float(row[i])
            y = float(row[i + 1])
            positions.append((x, y))
        all_positions.append(positions)

# Plot the robot and end-effector path
plt.figure()

# Plot obstacles
obstacles = [
    (1.0, 1.0, 2.0, 2.0),  # Rectangular obstacle 1
    (3.0, 3.0, 4.0, 4.0),  # Rectangular obstacle 2
    (2.0, 0.5, 3.0, 1.5),  # Additional rectangular obstacle 3
    (1.5, 2.5, 2.5, 3.5)   # Additional rectangular obstacle 4
]
for (x_min, y_min, x_max, y_max) in obstacles:
    plt.fill([x_min, x_max, x_max, x_min, x_min],
             [y_min, y_min, y_max, y_max, y_min],
             color='red', alpha=0.5)

# Define link width
link_width = 0.1

# Plot start state in blue
start_positions = all_positions[0]
for j in range(len(start_positions) - 1):
    start = start_positions[j]
    end = start_positions[j + 1]
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = np.sqrt(dx**2 + dy**2)
    angle = np.arctan2(dy, dx) * 180 / np.pi

    # Create a rectangle representing the link
    link_rect = patches.Rectangle(
        (start[0], start[1]), length, link_width,
        angle=angle, linewidth=1, edgecolor='blue', facecolor='blue', alpha=0.7, label='Start State' if j == 0 else ""
    )
    plt.gca().add_patch(link_rect)

# Plot joints for start state
xs, ys = zip(*start_positions)
plt.plot(xs, ys, marker='o', color='black', linestyle='')

# Plot middle states in light red (1 out of every 7 waypoints)
for i in range(1, len(all_positions) - 1, 7):
    middle_positions = all_positions[i]
    for j in range(len(middle_positions) - 1):
        start = middle_positions[j]
        end = middle_positions[j + 1]
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = np.sqrt(dx**2 + dy**2)
        angle = np.arctan2(dy, dx) * 180 / np.pi

        # Create a rectangle representing the link
        link_rect = patches.Rectangle(
            (start[0], start[1]), length, link_width,
            angle=angle, linewidth=1, edgecolor='lightcoral', facecolor='lightcoral', alpha=0.5, label='Middle State' if j == 0 and i == 1 else ""
        )
        plt.gca().add_patch(link_rect)

# Plot end state in green
end_positions = all_positions[-1]
for j in range(len(end_positions) - 1):
    start = end_positions[j]
    end = end_positions[j + 1]
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = np.sqrt(dx**2 + dy**2)
    angle = np.arctan2(dy, dx) * 180 / np.pi

    # Create a rectangle representing the link
    link_rect = patches.Rectangle(
        (start[0], start[1]), length, link_width,
        angle=angle, linewidth=1, edgecolor='green', facecolor='green', alpha=0.7, label='End State' if j == 0 else ""
    )
    plt.gca().add_patch(link_rect)

# Plot joints for end state
xs, ys = zip(*end_positions)
plt.plot(xs, ys, marker='o', color='black', linestyle='')

plt.xlabel('X')
plt.ylabel('Y')
plt.title('3R Robot Start, Middle, and End States')
plt.legend(loc='best')
plt.grid()
plt.axis('equal')
plt.show()


import re
import matplotlib.pyplot as plt
import numpy as np
import math
import os

# ======= CONFIGURATION =======
PLOT_3D = True  # Set to True for 3D camera trajectory
log_file = "/home/james/projects/Event_Based_Navigation/Software/logs/full_log.txt"
test_file = "/home/james/projects/Event_Based_Navigation/Testing/TestingLocations.txt"
# =============================

# ========== CAMERA TRAJECTORY ==========
# Pattern for camera trajectory
position_pattern = re.compile(r"Global Position:\s*\[([-.\d]+),\s*([-.\d]+),\s*([-.\d]+)\]")

# Extract camera positions
x_positions, y_positions, z_positions = [], [], []
with open(log_file, "r") as file:
    for line in file:
        match = position_pattern.search(line)
        if match:
            x, y, z = map(float, match.groups())
            x_positions.append(x)
            y_positions.append(y)
            z_positions.append(z)

x_positions = np.array(x_positions)
y_positions = np.array(y_positions)
z_positions = np.array(z_positions)

# Color gradient for time progression
color_values = np.linspace(0, 1, len(x_positions))
cmap = plt.cm.plasma
colors = cmap(color_values)

# ========== BUTTON PRESS PLOT ==========
# Load test data
with open(test_file, "r") as file:
    test_data = file.read()

with open(log_file, "r") as file:
    log_data = file.read()

patternTests = re.compile(r"\((\d+), (\d+)\)")
patternLogs = re.compile(r"(\d{2}:\d{2}:\d{2}) .*?CM: Button Pressed at pose: \((-?\d+),(-?\d+),-?\d+\)")
log_points = patternLogs.findall(log_data)
log_points = [(int(x), int(y), timestamp) for timestamp, x, y in log_points]

test_sections = test_data.split("Test ")[1:]
test = test_sections[0]  # Only plotting first test section for now

lines = test.strip().split("\n")
test_name = lines[0].strip()
points = patternTests.findall(test)
points = [(int(x), int(y)) for x, y in points]

# Matching log data
test_log_points = log_points[:len(points)]
test_x, test_y = zip(*points)
log_x, log_y, timestamps = zip(*test_log_points)
log_corrected_x = np.array(log_x) - log_x[0]
log_corrected_y = np.array(log_y) - log_y[0]

# ========== METRIC CALCULATIONS ==========
errors = []
for tx, ty, lx, ly in zip(test_x, test_y, log_corrected_x, log_corrected_y):
    error = math.sqrt((lx - tx) ** 2 + (ly - ty) ** 2)
    errors.append(error)

avg_error = np.mean(errors)
max_error = np.max(errors)
final_drift = math.sqrt((log_corrected_x[-1] - test_x[-1])**2 + (log_corrected_y[-1] - test_y[-1])**2)

print("\n====== PERFORMANCE METRICS ======")
print(f"Average Error Over Time: {avg_error:.2f} cm")
print(f"Max Error: {max_error:.2f} cm")
print(f"Final Drift: {final_drift:.2f} cm")
print("=================================")

# ========== PLOT COMBINED FIGURE ==========
fig = plt.figure(figsize=(14, 6))

# Subplot 1: Camera Trajectory
if PLOT_3D:
    ax1 = fig.add_subplot(121, projection="3d")
    sc = ax1.scatter(x_positions, y_positions, z_positions, c=color_values, cmap=cmap, marker="o")
    ax1.set_zlabel("Z (m)")
    ax1.set_title("3D Camera Trajectory")
else:
    ax1 = fig.add_subplot(121)
    sc = ax1.scatter(x_positions, y_positions, c=color_values, cmap=cmap, marker="o")
    ax1.set_title("2D Camera Trajectory")

ax1.set_xlabel("X (m)")
ax1.set_ylabel("Y (m)")
ax1.grid()
fig.colorbar(sc, ax=ax1, label="Time Progression")

# Subplot 2: Button Press Plot
ax2 = fig.add_subplot(122)
error_lines = []
for tx, ty, lx, ly, timestamp in zip(test_x, test_y, log_corrected_x, log_corrected_y, timestamps):
    line, = ax2.plot([tx, lx], [ty, ly], 'r:', alpha=0.8)
    error_lines.append(line)
    ax2.annotate(timestamp, (lx + 2, ly + 2), fontsize=12, fontweight='bold', alpha=0.9)

expected_plot = ax2.scatter(test_x, test_y, marker='o', color='b', label='Expected Positions')
logged_plot = ax2.scatter(log_corrected_x, log_corrected_y, marker='x', color='g', label='Log Corrected')

handles = [error_lines[0], expected_plot, logged_plot]
labels = ['Error Line', 'Expected Positions', 'Logged Positions']
ax2.legend(handles, labels)

ax2.set_xlabel('X (cm)')
ax2.set_ylabel('Y (cm)')
ax2.set_title(f'Test {test_name} vs Button Press Logs')
ax2.grid(True)

# Finalize
plt.tight_layout()
plt.show()

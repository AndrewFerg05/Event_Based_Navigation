import re
import matplotlib.pyplot as plt
import numpy as np

# ======= CONFIGURATION =======
PLOT_3D = True  # Set to False for 2D plot (X-Y only)
log_file = "/home/james/projects/Event_Based_Navigation/Software/logs/full_log.txt"
# =============================

# Regular expression to extract position
#position_pattern = re.compile(r"Global Position: \[([-.\d]+), ([-.\d]+), ([-.\d]+)\]")
position_pattern = re.compile(r"Global Position:\s+([-.\d]+)\s+([-.\d]+)\s+([-.\d]+)")

# Lists to store position data
x_positions = []
y_positions = []
z_positions = []

# Read and parse log file
with open(log_file, "r") as file:
    for line in file:
        match = position_pattern.search(line)
        if match:
            x, y, z = map(float, match.groups())  # Convert extracted strings to floats
            x_positions.append(x)
            y_positions.append(y)
            z_positions.append(z)

# Convert lists to numpy arrays
x_positions = np.array(x_positions)
y_positions = np.array(y_positions)
z_positions = np.array(z_positions)

# Check if data was extracted
if len(x_positions) == 0:
    print("No valid position data found in the log file.")
    exit()

# Generate colors based on position index
color_values = np.linspace(0, 1, len(x_positions))  # Normalize indices from 0 to 1
cmap = plt.cm.plasma  # Choose a colormap (plasma, viridis, jet, etc.)
colors = cmap(color_values)  # Apply colormap

# Plot camera trajectory
fig = plt.figure(figsize=(10, 6))

if PLOT_3D:
    ax = fig.add_subplot(111, projection="3d")
    #ax.plot(x_positions, y_positions, z_positions, marker="o", linestyle="-", color="b", label="Camera Path")
    sc = ax.scatter(x_positions, y_positions, z_positions, c=color_values, cmap=cmap, marker="o")
    ax.set_zlabel("Z Position (m)")
    ax.set_title("3D Camera Trajectory from OpenVINS Logs (Colour-coded by time)")
else:
    ax = fig.add_subplot(111)
    #ax.plot(x_positions, y_positions, marker="o", linestyle="-", color="b", label="Camera Path")
    sc = ax.scatter(x_positions, y_positions, c=color_values, cmap=cmap, marker="o")
    ax.set_title("2D Camera Trajectory (X-Y) from OpenVINS Logs (Colour-coded by time)")

# Labels and grid
ax.set_xlabel("X Position (m)")
ax.set_ylabel("Y Position (m)")
ax.legend()
ax.grid()

# Add color bar to indicate time progression
cbar = plt.colorbar(sc)
cbar.set_label("Time Progression (Start → End)")

# Show plot
plt.show()

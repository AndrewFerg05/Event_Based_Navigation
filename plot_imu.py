import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load IMU Data
df = pd.read_csv("/home/james/projects/Event_Based_Navigation/Software/build/imu_data.csv", names=["timestamp", "ax", "ay", "az", "gx", "gy", "gz"])

# Plot acceleration
plt.figure()
plt.plot(df["timestamp"], df["ax"], label="Acc X")
plt.plot(df["timestamp"], df["ay"], label="Acc Y")
plt.plot(df["timestamp"], df["az"], label="Acc Z")
plt.xlabel("Time")
plt.ylabel("Acceleration (m/s^2)")
plt.legend()
plt.title("IMU Accelerometer Data")

# Plot gyroscope
plt.figure()
plt.plot(df["timestamp"], df["gx"], label="Gyro X")
plt.plot(df["timestamp"], df["gy"], label="Gyro Y")
plt.plot(df["timestamp"], df["gz"], label="Gyro Z")
plt.xlabel("Time")
plt.ylabel("Angular Velocity (rad/s)")
plt.legend()
plt.title("IMU Gyroscope Data")

plt.show()


# Load Camera Pose Data
camera_data = np.loadtxt("/home/james/projects/Event_Based_Navigation/Software/build/camera_pose.csv", delimiter=",")

# Plot 3D trajectory
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(camera_data[:, 0], camera_data[:, 1], camera_data[:, 2], label="Camera Trajectory")
ax.set_xlabel("X Position (m)")
ax.set_ylabel("Y Position (m)")
ax.set_zlabel("Z Position (m)")
plt.title("Camera Trajectory")
plt.legend()
plt.show()
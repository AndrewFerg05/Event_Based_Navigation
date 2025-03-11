import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Configure Serial Port (Change COMx to your ESP32 port)
ser = serial.Serial('COM5', 9600, timeout=1)

# Lists to store x, y values
x_data, y_data = [], []

def read_serial():
    """Read and parse serial data"""
    try:
        line = ser.readline().decode().strip()  # Read a line from Serial
        if line:
            parts = line.split(',')  # Assuming format "10,20"
            if len(parts) == 2:
                x, y = float(parts[0]), float(parts[1])
                return x, y
    except Exception as e:
        print(f"Error reading serial: {e}")
    return None

def update(frame):
    """Update function for animation"""
    data = read_serial()
    if data:
        x, y = data
        x_data.append(x)
        y_data.append(y)

        ax.clear()
        ax.plot(x_data, y_data, marker='o', linestyle='-')

        # Set fixed axis limits
        ax.set_xlim(-1000, 4000)
        ax.set_ylim(-1000, 4000)

        ax.set_xlabel("X Position")
        ax.set_ylabel("Y Position")
        ax.set_title("Live XY Plot")

        # Add grid
        ax.grid(True, linestyle='--', alpha=0.6)

# Set up plot
fig, ax = plt.subplots()
ani = animation.FuncAnimation(fig, update, interval=100)  # Update every 100ms

plt.show()

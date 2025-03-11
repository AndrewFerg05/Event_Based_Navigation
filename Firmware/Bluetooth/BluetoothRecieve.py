import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Configure Serial Port (Change COMx to your ESP32 port)
ser = serial.Serial('COM4', 9600, timeout=1)

# Lists to store x, y values and heading
x_data, y_data, heading_data = [], [], []

def read_serial():
    """Read and parse serial data"""
    try:
        line = ser.readline().decode().strip()  # Read a line from Serial
        if line:
            parts = line.split(',')  # Assuming format "x, y, heading"
            if len(parts) == 3:  # We expect three values: x, y, heading
                x, y, heading = float(parts[0]), float(parts[1]), float(parts[2])
                return x, y, heading
    except Exception as e:
        print(f"Error reading serial: {e}")
    return None

def update(frame):
    """Update function for animation"""
    data = read_serial()
    if data:
        x, y, heading = data
        x_data.append(x)
        y_data.append(y)
        heading_data.append(heading)

        ax.clear()  # Clear the previous plot
        ax.plot(x_data, y_data, marker='o', linestyle='-', label="XY Position")

        # Set fixed axis limits
        ax.set_xlim(-1000, 4000)
        ax.set_ylim(-1000, 4000)

        ax.set_xlabel("X Position")
        ax.set_ylabel("Y Position")

        # Update the title to include the current heading value
        ax.set_title(f"Live XY Plot (Heading: {heading:.2f}°)")

        # Add grid
        ax.grid(True, linestyle='--', alpha=0.6)
        ax.legend()

# Set up plot
fig, ax = plt.subplots()
ani = animation.FuncAnimation(fig, update, interval=100)  # Update every 100ms

plt.show()


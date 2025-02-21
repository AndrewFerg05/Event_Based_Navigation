import sys
import cv2
import numpy as np
import time
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QImage, QPixmap, QFont
from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget, QGridLayout, QGroupBox
import socket
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt

running = True

def receiveData():
    buffer = b""
    data_size = None

    while True:
        # Receive data from the sender
        data, _ = sock.recvfrom(MAX_PACKET_SIZE)
        buffer += data

        # Parse the header (id and size) if not already done
        if data_size is None and len(buffer) >= 8:
            data_id = int.from_bytes(buffer[:4], byteorder='little')
            data_size = int.from_bytes(buffer[4:8], byteorder='little')
            buffer = buffer[8:]  # Remove the header from the buffer
            print(f"Receiving frame ID: {data_id}, size: {data_size} bytes")
            if data_id > 3:
                print('Invalid Data')
                return None

        # Check if all the data has been received
        if data_size is not None and len(buffer) >= data_size:
            # Extract the complete data
            sent_data = buffer[:data_size]

            if data_id < 2:     # Frame data type
                # Decode the frame using OpenCV
                encoded_frame = np.frombuffer(sent_data, dtype=np.uint8)
                frame = cv2.imdecode(encoded_frame, cv2.IMREAD_COLOR)

                if frame is None:
                    print("Failed to decode frame.")
                    return
                else:
                    return data_id, data_size, frame
            else:               # Status data type
                try:
                    if data_id == 2:
                        x = int.from_bytes(buffer[:4], byteorder='little', signed=True)
                        y = int.from_bytes(buffer[4:8], byteorder='little', signed=True)
                        z = int.from_bytes(buffer[8:12], byteorder='little', signed=True)
                        yaw = int.from_bytes(buffer[12:], byteorder='little', signed=True)
                        pitch = int.from_bytes(buffer[12:], byteorder='little', signed=True)
                        roll = int.from_bytes(buffer[12:], byteorder='little', signed=True)

                        return data_id, data_size, x, y, z, yaw, pitch, roll
                    elif data_id == 3:
                        x = int.from_bytes(buffer[:4], byteorder='little', signed=True)
                        y = int.from_bytes(buffer[4:8], byteorder='little', signed=True)
                        battery = int.from_bytes(buffer[8:12], byteorder='little', signed=True)
                        time = int.from_bytes(buffer[12:], byteorder='little', signed=True)

                        return data_id, data_size, x, y, battery, time
                    else:
                        return data_id, data_size
                except Exception as e:
                    print(f"Error parsing status data: {e}")
                    return None

class userInterface(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Rover Tracker")
        self.resize(800, 600)

        self.layout = QGridLayout()
        self.layout.setSpacing(0)
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.init_q1()
        self.init_q2()
        self.init_q3()
        self.init_q4()

        # Center alignment for text placeholders
        for label in [self.q1_border, self.q2_border, self.q3_border, self.q4_border]:
            label.setAlignment(Qt.AlignCenter)
            label.setStyleSheet("QGroupBox { border: 2px solid black; margin: -1px; padding: 0px; }")

        # Add widgets to the grid (row, column)
        self.layout.addWidget(self.q1_border, 0, 0)  # Top-left
        self.layout.addWidget(self.q2_border, 0, 1)  # Top-right
        self.layout.addWidget(self.q3_border, 1, 0)  # Bottom-left
        self.layout.addWidget(self.q4_border, 1, 1)  # Bottom-right
        self.setLayout(self.layout)

    def init_q1(self):
        # Create a group box for Quarter 1
        self.q1_border = QGroupBox()

        # Style the title of the group box
        self.q1_title = QLabel("Status")
        title_font = QFont("Arial", 14, QFont.Bold)
        title_font.setUnderline(True)
        self.q1_title.setFont(title_font)
        self.q1_title.setAlignment(Qt.AlignCenter)  # Center-align the title

        # Event pipeline labels (Green)
        self.label_eventCoord = QLabel("Event Pipeline Position: (0.0, 0.0, 0.0)")
        self.label_eventAngle = QLabel("Event Pipeline Heading: (0.0, 0.0, 0.0)")
        self.label_eventVel = QLabel("Event Pipeline Speed: 0.0")

        self.label_eventCoord.setStyleSheet("color: green;")
        self.label_eventAngle.setStyleSheet("color: green;")
        self.label_eventVel.setStyleSheet("color: green;")

        # Alt pipeline labels (Blue)
        self.label_altCoord = QLabel("Alt. Pipeline Position: (0.0, 0.0)")
        self.label_altAngle = QLabel("Alt. Pipeline Heading: (0.0)")
        self.label_altVel = QLabel("Alt. Pipeline Speed: 0.0")

        self.label_altCoord.setStyleSheet("color: blue;")
        self.label_altAngle.setStyleSheet("color: blue;")
        self.label_altVel.setStyleSheet("color: blue;")

        # Other labels (Black)
        self.label_battery = QLabel("Rover Battery: 100%")
        self.label_connection = QLabel("WiFi Connection: Strong")

        self.label_battery.setStyleSheet("color: black;")
        self.label_connection.setStyleSheet("color: black;")

        # Set font for all labels
        info_font = QFont("Arial", 10)  # Set font size for labels
        self.label_eventCoord.setFont(info_font)
        self.label_eventAngle.setFont(info_font)
        self.label_eventVel.setFont(info_font)
        self.label_altCoord.setFont(info_font)
        self.label_altAngle.setFont(info_font)
        self.label_altVel.setFont(info_font)
        self.label_battery.setFont(info_font)
        self.label_connection.setFont(info_font)

        # Create a vertical layout for Quarter 1
        vbox = QVBoxLayout()
        vbox.addWidget(self.q1_title)  # Add title at the top
        vbox.addWidget(self.label_eventCoord)
        vbox.addWidget(self.label_eventAngle)
        vbox.addWidget(self.label_eventVel)
        vbox.addWidget(self.label_altCoord)
        vbox.addWidget(self.label_altAngle)
        vbox.addWidget(self.label_altVel)
        vbox.addWidget(self.label_battery)
        vbox.addWidget(self.label_connection)
        vbox.setAlignment(Qt.AlignTop)  # Keep layout aligned to the top

        # Set the layout for Quarter 1
        self.q1_border.setLayout(vbox)

    def init_q2(self):
        self.q2_border = QGroupBox("Position Tracker")

        # Matplotlib Figure & Canvas
        self.figure, self.ax = plt.subplots()
        self.canvas = FigureCanvas(self.figure)

        # Data storage (deque keeps last 20 points)
        self.x_data_pi = []  # Event Pipeline (Pi)
        self.y_data_pi = []

        self.x_data_esp = []  # Alt Pipeline (ESP)
        self.y_data_esp = []

        # Graph layout
        vbox = QVBoxLayout()
        vbox.addWidget(self.canvas)
        self.q2_border.setLayout(vbox)

        # Initialize graph
        self.ax.set_title("Live Position Tracking")
        self.ax.set_xlabel("X Position")
        self.ax.set_ylabel("Y Position")
        self.ax.grid(True)
        self.ax.legend(["Event Pipeline (Pi)", "Alt Pipeline (ESP)"])

    def init_q3(self):
        self.q3_border = QGroupBox()
        self.q3_label = QLabel("Frames")
        vbox = QVBoxLayout()
        vbox.addWidget(self.q3_label)
        vbox.setAlignment(Qt.AlignCenter)
        self.q3_border.setLayout(vbox)

    def init_q4(self):
        self.q4_border = QGroupBox()
        self.q4_label = QLabel("Events")
        vbox = QVBoxLayout()
        vbox.addWidget(self.q4_label)
        vbox.setAlignment(Qt.AlignCenter)
        self.q4_border.setLayout(vbox)

    def update_PiStatus(self, event_coordinates, event_angle, event_velocity):
        # Update the event pipeline labels (Green)
        self.label_eventCoord.setText(f"Event Pipeline Position: {event_coordinates}")
        self.label_eventAngle.setText(f"Event Pipeline Heading: {event_angle}")
        self.label_eventVel.setText(f"Event Pipeline Speed: {event_velocity}")

    def update_ESPStatus(self, alt_coordinates, alt_angle, alt_velocity, battery, connection):
        # Update the alt pipeline labels (Blue)
        self.label_altCoord.setText(f"Alt. Pipeline Position: {alt_coordinates}")
        self.label_altAngle.setText(f"Alt. Pipeline Heading: {alt_angle}")
        self.label_altVel.setText(f"Alt. Pipeline Speed: {alt_velocity}")

        # Update battery and connection status (Black)
        self.label_battery.setText(f"Rover Battery: {battery}%")
        self.label_connection.setText(f"WiFi Connection: {connection}")

    def display_error(self, label, error_message):
        # Display an error message on the given label
        label.setText(error_message)
        label.setStyleSheet("color: red; font-weight: bold; font-size: 20px")

    def display_frame(self, label, frame):
        # Convert the frame to RGB format and then to QImage
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = frame.shape
        bytes_per_line = ch * w
        qimg = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimg)
        label.setPixmap(pixmap)
        label.setScaledContents(True)

    def closeEvent(self, event):
        print("Closing user interface...")
        global running
        running = False
        # Release video capture on close
        super().closeEvent(event)

    def update_graph(self, x, y, source):
        if source == "Pi":
            self.x_data_pi.append(x)
            self.y_data_pi.append(y)
        elif source == "ESP":
            self.x_data_esp.append(x)
            self.y_data_esp.append(y)

        self.ax.clear()

        # Plot both pipelines
        self.ax.plot(self.x_data_pi, self.y_data_pi, marker="o", linestyle="-", color="green",
                     label="Event Pipeline (Pi)")
        self.ax.plot(self.x_data_esp, self.y_data_esp, marker="s", linestyle="-", color="blue",
                     label="Alt Pipeline (ESP)")

        # Redraw the graph
        self.ax.set_title("Live Position Tracking")
        self.ax.set_xlabel("X Position")
        self.ax.set_ylabel("Y Position")
        self.ax.grid(True)
        self.ax.legend()

        self.canvas.draw()

class timingTracker():
    def __init__(self, to):
        self.startTime = time.time()
        self.timeout = to

    def checkElapse(self):
        if self.startTime + self.timeout > time.time():
            return False    #Time has not elapsed yet
        else:
            return True

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = userInterface()
    window.show()

    # Configuration
    PC_IP = "0.0.0.0"  # Listen on all network interfaces
    PC_PORT = 5005  # Port to receive video data
    MAX_PACKET_SIZE = 65507  # Maximum UDP packet size

    # Initialize socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((PC_IP, PC_PORT))
    sock.settimeout(0.1)

    # Buffer to reassemble incoming data
    buffer = b""
    frame_size = None

    # Initialise timing trackers
    eventStream = timingTracker(1)
    frameStream = timingTracker(1)
    statusPiStream = timingTracker(1)
    statusESPStream = timingTracker(1)

    print(f"Listening for video stream on {PC_IP}:{PC_PORT}...")

    try:
        while running:
            # Indicate if no message
            if statusPiStream.checkElapse() == True:
                window.update_PiStatus("ERROR", "ERROR", "ERROR")
                cv2.waitKey(1)

            if statusESPStream.checkElapse() == True:
                window.update_ESPStatus("ERROR", "ERROR", "ERROR", "ERROR", "ERROR")
                cv2.waitKey(1)

            if frameStream.checkElapse() == True:
                window.display_error(window.q3_label, "Frames Lost!")
                cv2.waitKey(1)

            if eventStream.checkElapse() == True:
                window.display_error(window.q4_label, "Events Lost!")
                cv2.waitKey(1)

            try:
                frame_data = receiveData()

                # Extract data ID and size
                data_id = frame_data[0]
                data_size = frame_data[1]

                # Display the frame with its video ID in a different window for each ID
                if data_id == 0:
                    frame = frame_data[2]
                    window.display_frame(window.q3_label, frame)
                    cv2.waitKey(1)
                    frameStream.startTime = time.time()
                elif data_id == 1:
                    frame = frame_data[2]
                    window.display_frame(window.q4_label, frame)
                    cv2.waitKey(1)
                    eventStream.startTime = time.time()
                elif data_id == 2:
                    x = frame_data[2]
                    y = frame_data[3]
                    z = frame_data[4]
                    yaw = frame_data[5]
                    pitch = frame_data[6]
                    roll = frame_data[7]
                    window.update_PiStatus(f"({x}, {y}, {z})", f"({yaw}, {pitch}, {roll})", str(0))
                    window.update_graph(x, y, "Pi")  # Update the graph with new position
                    cv2.waitKey(1)
                    statusPiStream.startTime = time.time()
                elif data_id == 3:
                    x = frame_data[2]
                    y = frame_data[3]
                    a = frame_data[4]
                    b = frame_data[5]
                    window.update_ESPStatus(f"({x}, {y})", str(a), str(b), str(0), str(1))
                    window.update_graph(x, y, "ESP")
                    cv2.waitKey(1)
                    statusESPStream.startTime = time.time()

                else:
                    print(f"Unknown Data ID: {data_id}")
            except:
                continue

    except KeyboardInterrupt:
        print("Keyboard interrupt...")
    finally:
        print("Cleaning up...")
        sock.close()

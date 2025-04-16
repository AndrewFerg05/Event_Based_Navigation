import sys
import cv2
import numpy as np
import time
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QImage, QPixmap, QFont
from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget, QGridLayout, QGroupBox, QPushButton
import socket
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt

import csv

running = True

orig_graphSizeX = [-100, 500]
orig_graphSizeY = [-100, 500]
graphSizeX = orig_graphSizeX
graphSizeY = orig_graphSizeY

def append_data_to_csv(x1, y1, x2, y2):
    results = "log_UI.csv"

    max_size = max(len(x1), len(y1), len(x2), len(y2))
    fill_value = 0

    print(max_size)
    if max_size == 0:
        max_size = 1

    x1.extend([fill_value] * (max_size - len(x1)))
    y1.extend([fill_value] * (max_size - len(y1)))
    x2.extend([fill_value] * (max_size - len(x2)))
    y2.extend([fill_value] * (max_size - len(y2)))

    print("Opening file...")
    with open(results, 'a', newline='') as file:
        print("File opened")
        writer = csv.writer(file)

        # Write time of test
        writer.writerow([time.strftime("%Y-%m-%d %H:%M:%S")])

        # Write the column headers
        writer.writerow(["x Pi", "y Pi", "x ESP", "y ESP"])

        # Write the data rows
        for i in range(max_size):
            writer.writerow([x1[i], y1[i], x2[i], y2[i]])

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
            if data_id > 4:
                print('Invalid Data')
                return None

        # Check if all the data has been received
        if data_size is not None and len(buffer) >= data_size:
            # Extract the complete data
            sent_data = buffer[:data_size]

            if data_id < 3:     # Frame data type
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
                    if data_id == 4:        #Pi
                        # This will come together, if not will all be -1 (unlikely that all are -1 at once)
                        x = int.from_bytes(buffer[:4], byteorder='little', signed=True)
                        y = int.from_bytes(buffer[4:8], byteorder='little', signed=True)
                        z = int.from_bytes(buffer[8:12], byteorder='little', signed=True)
                        yaw = int.from_bytes(buffer[12:16], byteorder='little', signed=True)
                        pitch = int.from_bytes(buffer[16:20], byteorder='little', signed=True)
                        roll = int.from_bytes(buffer[20:24], byteorder='little', signed=True)
                        vel = int.from_bytes(buffer[24:28], byteorder='little', signed=True)

                        # If not the thing to update will be set to -1
                        state = int.from_bytes(buffer[28:32], byteorder='little', signed=True)

                        # Features, expected position in x,y,z again set to -1 if not to be updated
                        feat_x = int.from_bytes(buffer[32:36], byteorder='little', signed=True)
                        feat_y = int.from_bytes(buffer[36:40], byteorder='little', signed=True)
                        feat_z = int.from_bytes(buffer[40:], byteorder='little', signed=True)

                        return data_id, data_size, x, y, z, yaw, pitch, roll, vel, state, feat_x, feat_y, feat_z
                    elif data_id == 3:              #ESP Position
                        x = int.from_bytes(buffer[:4], byteorder='little', signed=True)
                        y = int.from_bytes(buffer[4:8], byteorder='little', signed=True)
                        heading = int.from_bytes(buffer[8:12], byteorder='little', signed=True)
                        vel = int.from_bytes(buffer[12:16], byteorder='little', signed=True)
                        state = int.from_bytes(buffer[16:], byteorder='little', signed=True)

                        return data_id, data_size, x, y, heading, vel, state
                    elif data_id == 4:      #ESP Debug
                        RCConnected = int.from_bytes(buffer[:4], byteorder='little', signed=True)
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
        self.init_s1()
        self.init_s2()
        self.init_s3()
        self.init_s4()
        self.init_s5()
        self.init_s6()

        # Center alignment for text placeholders
        for label in [self.s1_border, self.s2_border, self.s3_border, self.s4_border, self.s5_border, self.s6_border,]:
            label.setAlignment(Qt.AlignCenter)
            label.setStyleSheet("QGroupBox { border: 2px solid black; margin: -1px; padding: 0px; }")

        # Add widgets to the grid (row, column)
        self.layout.addWidget(self.s1_border, 0, 0)  # Top-left
        self.layout.addWidget(self.s2_border, 0, 1)  # Top-middle
        self.layout.addWidget(self.s3_border, 0, 2)  # Top-right
        self.layout.addWidget(self.s4_border, 1, 0)  # Bottom-left
        self.layout.addWidget(self.s5_border, 1, 1)  # Bottom-middle
        self.layout.addWidget(self.s6_border, 1, 2)  # Bottom-right
        self.setLayout(self.layout)

    def init_s1(self):
        # Create a group box for Quarter 1
        self.s1_border = QGroupBox()

        # Style the title of the group box
        self.s1_title = QLabel("Pi Status")
        title_font = QFont("Arial", 14, QFont.Bold)
        title_font.setUnderline(True)
        self.s1_title.setFont(title_font)
        self.s1_title.setStyleSheet("color: red;")
        self.s1_title.setAlignment(Qt.AlignCenter)  # Center-align the title

        # Event pipeline labels
        self.label_eventCoord = QLabel("Event Pipeline Position: (0.0, 0.0, 0.0)")
        self.label_eventAngle = QLabel("Event Pipeline Heading: (0.0, 0.0, 0.0)")
        self.label_eventVel = QLabel("Event Pipeline Speed: 0.0")

        self.label_piState = QLabel("Pi State: Not Started (-1)")
        self.label_piTime = QLabel("Time since last receive: -1")
        self.label_frameTime = QLabel("Time since last camera frame: -1")
        self.label_eventTime = QLabel("Time since last event frame: -1")
        self.label_augmentedTime = QLabel("Time since last augmented frame: -1")

        self.reset_pi_button = QPushButton("Reset Pi Data")
        self.reset_pi_button.clicked.connect(self.reset_pi_data)

        # Set font for all labels
        info_font = QFont("Arial", 10)
        self.label_eventCoord.setFont(info_font)
        self.label_eventAngle.setFont(info_font)
        self.label_eventVel.setFont(info_font)

        self.label_piState.setFont(info_font)
        self.label_piTime.setFont(info_font)
        self.label_frameTime.setFont(info_font)
        self.label_eventTime.setFont(info_font)
        self.label_augmentedTime.setFont(info_font)

        # Create a vertical layout for Quarter 1
        vbox = QVBoxLayout()
        vbox.addWidget(self.s1_title)  # Add title at the top
        vbox.addWidget(self.label_eventCoord)
        vbox.addWidget(self.label_eventAngle)
        vbox.addWidget(self.label_eventVel)

        vbox.addWidget(self.label_piState)
        vbox.addWidget(self.label_piTime)
        vbox.addWidget(self.label_frameTime)
        vbox.addWidget(self.label_eventTime)
        vbox.addWidget(self.label_augmentedTime)

        vbox.addWidget(self.reset_pi_button)
        vbox.setAlignment(Qt.AlignTop)  # Keep layout aligned to the top

        # Set the layout for Quarter 1
        self.s1_border.setLayout(vbox)

    def init_s2(self):
        self.s2_border = QGroupBox()

        # Matplotlib Figure & Canvas
        self.figure, self.ax = plt.subplots()
        self.canvas = FigureCanvas(self.figure)

        self.x_data_pi = []  # Event Pipeline (Pi)
        self.y_data_pi = []

        self.x_data_esp = []  # Alt Pipeline (ESP)
        self.y_data_esp = []

        self.x_data_feat = []  # Feature points (Pi)
        self.y_data_feat = []
        self.z_data_feat = []

        self.log_button = QPushButton("Log Graph Data")
        self.log_button.clicked.connect(self.log_graph)

        # Graph layout
        vbox = QVBoxLayout()
        vbox.addWidget(self.canvas)
        vbox.addWidget(self.log_button)
        self.s2_border.setLayout(vbox)

        # Initialize graph
        self.ax.set_title("Live Position Tracking")
        self.ax.set_xlabel("X Position")
        self.ax.set_ylabel("Y Position")
        self.ax.set_xlim(graphSizeX)
        self.ax.set_ylim(graphSizeY)
        self.ax.grid(True)
        self.ax.legend(["Event Pipeline (Pi)", "Alt Pipeline (ESP)"])

    def init_s3(self):
        # Create a group box for Alt. Pipeline
        self.s3_border = QGroupBox("")

        # Style the title of the group box
        self.s3_title = QLabel("ESP32 Status")
        title_font = QFont("Arial", 14, QFont.Bold)
        title_font.setUnderline(True)
        self.s3_title.setFont(title_font)
        self.s3_title.setStyleSheet("color: blue;")
        self.s3_title.setAlignment(Qt.AlignCenter)  # Center-align the title

        # Alt pipeline labels
        self.label_altCoord = QLabel("Alt. Pipeline Position: (0.0, 0.0)")
        self.label_altAngle = QLabel("Alt. Pipeline Heading: (0.0)")
        self.label_altVel = QLabel("Alt. Pipeline Speed: 0.0")
        self.label_altState = QLabel("Alt. Pipeline State: Not Started (-1)")

        self.label_espTime = QLabel("fTime since last receive: 0.0")

        self.reset_esp_button = QPushButton("Reset ESP Data")
        self.reset_esp_button.clicked.connect(self.reset_esp_data)

        # Set font for Alt. Pipeline labels
        info_font = QFont("Arial", 10)
        self.label_altCoord.setFont(info_font)
        self.label_altAngle.setFont(info_font)
        self.label_altVel.setFont(info_font)
        self.label_altState.setFont(info_font)
        self.label_espTime.setFont(info_font)

        # Create a vertical layout for Alt. Pipeline
        vbox = QVBoxLayout()
        vbox.addWidget(self.s3_title)  # Add title at the top
        vbox.addWidget(self.label_altCoord)
        vbox.addWidget(self.label_altAngle)
        vbox.addWidget(self.label_altVel)
        vbox.addWidget(self.label_altState)
        vbox.addWidget(self.label_espTime)
        vbox.addWidget(self.reset_esp_button)
        vbox.setAlignment(Qt.AlignTop)

        # Set the layout for Alt. Pipeline
        self.s3_border.setLayout(vbox)

    def init_s4(self):
        self.s4_border = QGroupBox()
        self.s4_label = QLabel("Frames")
        vbox = QVBoxLayout()
        vbox.addWidget(self.s4_label)
        vbox.setAlignment(Qt.AlignCenter)
        self.s4_border.setLayout(vbox)

    def init_s5(self):
        self.s5_border = QGroupBox()
        self.s5_label = QLabel("Events")
        vbox = QVBoxLayout()
        vbox.addWidget(self.s5_label)
        vbox.setAlignment(Qt.AlignCenter)
        self.s5_border.setLayout(vbox)

    def init_s6(self):
        self.s6_border = QGroupBox()
        self.s6_label = QLabel("Augmented")
        vbox = QVBoxLayout()
        vbox.addWidget(self.s6_label)
        vbox.setAlignment(Qt.AlignCenter)
        self.s6_border.setLayout(vbox)

    def update_PiState(self, state):
        if state == 0:
            self.label_piState.setText(f"Pi State: Idle ({state})")
        elif state == 1:
            self.label_piState.setText(f"Pi State: Run ({state})")
        else:
            self.label_piState.setText(f"Pi State: Stop ({state})")

    def update_PiPose(self, event_coordinates, event_angle, event_velocity):
        # Update the event pipeline labels
        self.label_eventCoord.setText(f"Event Pipeline Position: {event_coordinates}")
        self.label_eventAngle.setText(f"Event Pipeline Heading: {event_angle}")
        self.label_eventVel.setText(f"Event Pipeline Speed: {event_velocity}")

    def update_ESPStatus(self, alt_coordinates, alt_head, alt_velocity, alt_state):
        # Update the alt pipeline labels (Blue)
        self.label_altCoord.setText(f"Alt. Pipeline Position: {alt_coordinates}")
        self.label_altAngle.setText(f"Alt. Pipeline Heading: {alt_head}")
        self.label_altVel.setText(f"Alt. Pipeline Speed: {alt_velocity}")

        if alt_state == 0:
            self.label_altState.setText(f"Alt. Pipeline State: Idle ({alt_state})")
        elif alt_state == 1:
            self.label_altState.setText(f"Alt. Pipeline State: Run ({alt_state})")
        else:
            self.label_altState.setText(f"Alt. Pipeline State: Stop ({alt_state})")

    def update_ESPTime(self, time):
        self.label_espTime.setText(f"Time since last receive: {time:.3f}")
        if time > 3:
            self.label_espTime.setStyleSheet("color: red;")
        else:
            self.label_espTime.setStyleSheet("color: black;")

    def update_PiTime(self, time):
        self.label_piTime.setText(f"Time since last receive: {time:.3f}")
        if time > 3:
            self.label_piTime.setStyleSheet("color: red;")
        else:
            self.label_piTime.setStyleSheet("color: black;")

    def update_frameTime(self, time):
        self.label_frameTime.setText(f"Time since last camera frame: {time:.3f}")
        if time > 3:
            self.label_frameTime.setStyleSheet("color: red;")
        else:
            self.label_frameTime.setStyleSheet("color: black;")

    def update_eventTime(self, time):
        self.label_eventTime.setText(f"Time since last event frame: {time:.3f}")
        if time > 3:
            self.label_eventTime.setStyleSheet("color: red;")
        else:
            self.label_eventTime.setStyleSheet("color: black;")

    def update_augmentedTime(self, time):
        self.label_augmentedTime.setText(f"Time since last augmented frame: {time:.3f}")
        if time > 3:
            self.label_augmentedTime.setStyleSheet("color: red;")
        else:
            self.label_augmentedTime.setStyleSheet("color: black;")

    def display_error(self, label, error_message):
        # Display an error message on the given label
        label.setText(error_message)
        label.setStyleSheet("color: red; font-weight: bold; font-size: 20px")

    def display_frame(self, label, frame):
        # Get the current window size
        window_width = self.width()
        window_height = self.height()

        # Define how much of the window the frame should take up
        frame_width = window_width // 3  # Assuming 3 frames in bottom half
        frame_height = window_height // 2  # Half of the window for frames

        # Ensure a minimum size for frames
        min_frame_width = 320
        min_frame_height = 240

        frame_width = max(frame_width, min_frame_width)
        frame_height = max(frame_height, min_frame_height)

        # Resize the frame to fit the computed size
        resized_frame = cv2.resize(frame, (frame_width, frame_height), interpolation=cv2.INTER_LINEAR)

        # Convert the frame to RGB format for Qt
        resized_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2RGB)
        h, w, ch = resized_frame.shape
        bytes_per_line = ch * w
        qimg = QImage(resized_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)

        # Convert to pixmap and set to label
        pixmap = QPixmap.fromImage(qimg)
        label.setPixmap(pixmap)

        # Ensure contents fill the label without distortion
        label.setScaledContents(False)

        # Prevent the window from being resized too small
        min_window_width = min_frame_width * 3  # Minimum width for 3 frames
        min_window_height = min_frame_height * 2  # Half window for info + frames

        self.setMinimumSize(min_window_width, min_window_height)

    def closeEvent(self, event):
        print("Closing user interface...")
        global running
        running = False
        # Release video capture on close
        super().closeEvent(event)

    def update_graph(self, x, y, z, source):
        if source == "Pi":
            self.x_data_pi.append(x)
            self.y_data_pi.append(y)
        elif source == "ESP":
            self.x_data_esp.append(x)
            self.y_data_esp.append(y)
        elif source == "Feature":
            self.x_data_feat.append(x)
            self.y_data_feat.append(y)
            self.z_data_feat.append(y)

        self.ax.clear()

        # Plot both pipelines
        self.ax.plot(self.x_data_pi, self.y_data_pi, marker="none", linestyle="-", color="red",
                     label="Event Pipeline (Pi)")
        self.ax.plot(self.x_data_esp, self.y_data_esp, marker="none", linestyle="--", color="blue",
                     label="Alt Pipeline (ESP)")

        # Redraw the graph
        self.ax.set_title("Live Position Tracking")
        self.ax.set_xlabel("X Position")
        self.ax.set_ylabel("Y Position")

        if source == "ESP" or source == "Pi":
            if graphSizeX[0] > x:
                graphSizeX[0] = x - 10
            if graphSizeX[1] < x:
                graphSizeX[1] = x + 10
            if graphSizeY[0] > y:
                graphSizeY[0] = y - 10
            if graphSizeY[1] < y:
                graphSizeY[1] = y + 10

        self.ax.set_xlim(graphSizeX)
        self.ax.set_ylim(graphSizeY)
        self.ax.grid(True)
        self.ax.legend()

        self.canvas.draw()

    def reset_pi_data(self):
        self.x_data_pi.clear()
        self.y_data_pi.clear()

        # Plot both pipelines
        self.ax.plot(self.x_data_pi, self.y_data_pi, marker="none", linestyle="-", color="red",
                     label="Event Pipeline (Pi)")

        # Also reclear all feature points

        # Redraw the graph
        self.ax.set_title("Live Position Tracking")
        self.ax.set_xlabel("X Position")
        self.ax.set_ylabel("Y Position")

        graphSizeX = orig_graphSizeX
        graphSizeY = orig_graphSizeY

        self.ax.set_xlim(graphSizeX)
        self.ax.set_ylim(graphSizeY)
        self.ax.grid(True)
        self.ax.legend()

        self.canvas.draw()

    def reset_esp_data(self):
        self.x_data_esp.clear()
        self.y_data_esp.clear()

        self.ax.plot(self.x_data_esp, self.y_data_esp, marker="none", linestyle="-", color="blue",
                     label="Alt Pipeline (ESP)")

        # Redraw the graph
        self.ax.set_title("Live Position Tracking")
        self.ax.set_xlabel("X Position")
        self.ax.set_ylabel("Y Position")

        graphSizeX = orig_graphSizeX
        graphSizeY = orig_graphSizeY

        self.ax.set_xlim(graphSizeX)
        self.ax.set_ylim(graphSizeY)
        self.ax.grid(True)
        self.ax.legend()

    def log_graph(self):
        print("logging")
        append_data_to_csv(self.x_data_pi, self.y_data_pi, self.x_data_esp, self.y_data_esp)
        print("logging done")

class timingTracker():
    def __init__(self):
        self.startTime = time.time()

    def checkElapse(self):
        return time.time() - self.startTime

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
    eventStream = timingTracker()
    frameStream = timingTracker()
    augmentedStream = timingTracker()
    statusPiStream = timingTracker()
    statusESPStream = timingTracker()

    updateGUITimings = timingTracker()

    print(f"Listening for data stream on {PC_IP}:{PC_PORT}...")

    try:
        while running:
            # Indicate time since last message receive
            if updateGUITimings.checkElapse() > 0.1:
                window.update_PiTime(statusPiStream.checkElapse())
                window.update_ESPTime(statusESPStream.checkElapse())
                window.update_frameTime(frameStream.checkElapse())
                window.update_eventTime(eventStream.checkElapse())
                window.update_augmentedTime(augmentedStream.checkElapse())
                cv2.waitKey(1)
                updateGUITimings.startTime = time.time()

            try:
                received_data = receiveData()

                # Extract data ID and size
                data_id = received_data[0]
                data_size = received_data[1]

                # Display the frame with its video ID in a different window for each ID
                if data_id == 0:
                    frame = received_data[2]
                    window.display_frame(window.s4_label, frame)
                    cv2.waitKey(1)
                    frameStream.startTime = time.time()
                elif data_id == 1:
                    frame = received_data[2]
                    window.display_frame(window.s5_label, frame)
                    cv2.waitKey(1)
                    eventStream.startTime = time.time()
                elif data_id == 2:
                    frame = received_data[2]
                    window.display_frame(window.s6_label, frame)
                    cv2.waitKey(1)
                    augmentedStream.startTime = time.time()
                elif data_id == 4:
                    x = received_data[2]
                    y = received_data[3]
                    z = received_data[4]
                    yaw = received_data[5]
                    pitch = received_data[6]
                    roll = received_data[7]
                    vel = received_data[8]
                    if (x == -1) and (y == -1) and (z == -1) and (yaw == -1) and (pitch == -1) and (roll == -1) and (vel == -1):  # Not a pose update
                        state = received_data[9]
                        if state != -1: # A state update
                            window.update_PiState(state)
                            # print("State Update")
                            cv2.waitKey(1)
                        else:           # A new feature location update
                            feat_x = received_data[10]
                            feat_y = received_data[11]
                            feat_z = received_data[12]
                            window.update_graph(feat_x, feat_y, 0, "Feature")
                            # print("Feature Update")
                            cv2.waitKey(1)
                    else:   # A pose update
                        window.update_PiPose(f"({x}, {y}, {z})", f"({yaw}, {pitch}, {roll})", f"{vel}")
                        window.update_graph(x, y, z, "Pi")  # Update the graph with new position
                        cv2.waitKey(1)


                    statusPiStream.startTime = time.time()
                elif data_id == 3:
                    x = received_data[2]
                    y = received_data[3]
                    head = received_data[4]
                    vel = received_data[5]
                    state = received_data[6]
                    window.update_ESPStatus(f"({x}, {y})", head, vel, state)
                    window.update_graph(x, y, 0, "ESP")
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

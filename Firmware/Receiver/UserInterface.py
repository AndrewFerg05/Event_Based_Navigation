import sys
import cv2
import numpy as np
import time
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QImage, QPixmap, QFont
from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget, QGridLayout, QGroupBox
import socket

running = True

def receiveData(setup):
    if setup == 0:
        setup =  1
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
                print('Invalid frame')
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
                    x = int.from_bytes(buffer[:4], byteorder='little')
                    y = int.from_bytes(buffer[4:8], byteorder='little')
                    battery = int.from_bytes(buffer[8:12], byteorder='little')
                    time = int.from_bytes(buffer[12:], byteorder='little')

                    return data_id, data_size, x, y, battery, time
                except Exception as e:
                    print(f"Error parsing status data: {e}")
                    return None




class userInterface(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Rover Tracker")
        self.resize(800, 600)

        self.layout = QGridLayout()
        self.init_quarter1()
        self.quarter2 = QLabel("Tracker")
        self.quarter3 = QLabel("Frames")
        self.quarter4 = QLabel("Events")

        # Center alignment for text placeholders
        for label in [self.quarter1, self.quarter2, self.quarter3, self.quarter4]:
            label.setAlignment(Qt.AlignCenter)

        # Add widgets to the grid (row, column)
        self.layout.addWidget(self.quarter1, 0, 0)  # Top-left
        self.layout.addWidget(self.quarter2, 0, 1)  # Top-right
        self.layout.addWidget(self.quarter3, 1, 0)  # Bottom-left
        self.layout.addWidget(self.quarter4, 1, 1)  # Bottom-right
        self.setLayout(self.layout)

    def init_quarter1(self):
        # Create a group box for Quarter 1
        self.quarter1 = QGroupBox("Status")

        # Style the title of the group box
        title_font = QFont("Arial", 12, QFont.Bold)
        title_font.setUnderline(True)
        self.quarter1.setFont(title_font)

        # Create labels for status information
        self.coordinates_label = QLabel("Coordinates: (0.0, 0.0)")
        self.battery_label = QLabel("Battery: 100%")
        self.connection_label = QLabel("Connection: Strong")

        # Style the labels
        info_font = QFont("Arial", 10)  # Set font size for labels
        info_font.setUnderline(False)
        self.coordinates_label.setFont(info_font)
        self.battery_label.setFont(info_font)
        self.connection_label.setFont(info_font)

        # Create a vertical layout for Quarter 1
        vbox = QVBoxLayout()
        vbox.addWidget(self.coordinates_label)
        vbox.addWidget(self.battery_label)
        vbox.addWidget(self.connection_label)

        # Set the layout for Quarter 1
        self.quarter1.setLayout(vbox)

    def update_status(self, coordinates, battery, connection):
        # Update the labels with dynamic data
        self.coordinates_label.setText(f"Coordinates: {coordinates}")
        self.battery_label.setText(f"Battery: {battery}%")
        self.connection_label.setText(f"Connection: {connection}")

    def display_error(self, label, error_message):
        # Display an error message on the given label
        label.setText(error_message)
        label.setStyleSheet("color: red; font-weight: bold;")

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
    statusStream = timingTracker(1)
    receiveCheck = timingTracker(2)

    print(f"Listening for video stream on {PC_IP}:{PC_PORT}...")

    try:
        while running:
            # Indicate if data has been lost
            if statusStream.checkElapse() == True:
                window.update_status("ERROR", "ERROR", "ERROR")
                cv2.waitKey(1)

            if frameStream.checkElapse() == True:
                window.display_error(window.quarter3, "Frames Lost!")
                cv2.waitKey(1)

            if eventStream.checkElapse() == True:
                window.display_error(window.quarter4, "Events Lost!")
                cv2.waitKey(1)

            try:
                frame_data = receiveData(0)

                # Extract video ID and frame
                video_id = frame_data[0]
                frame_size = frame_data[1]

                # Display the frame with its video ID in a different window for each ID
                if video_id == 0:
                    frame = frame_data[2]
                    window.display_frame(window.quarter3, frame)
                    cv2.waitKey(1)
                    frameStream.startTime = time.time()
                elif video_id == 1:
                    frame = frame_data[2]
                    window.display_frame(window.quarter4, frame)
                    cv2.waitKey(1)
                    eventStream.startTime = time.time()
                elif video_id == 2 or video_id == 3:
                    x = frame_data[2]
                    y = frame_data[3]
                    a = frame_data[4]
                    b = frame_data[5]
                    window.update_status("("+str(x)+","+str(y)+")",
                                         str(a),
                                         str(b))
                    cv2.waitKey(1)
                    statusStream.startTime = time.time()

                else:
                    print(f"Unknown Video ID: {video_id}")
            except:
                continue

    except KeyboardInterrupt:
        print("Keyboard interrupt...")
    finally:
        print("Cleaning up...")
        sock.close()

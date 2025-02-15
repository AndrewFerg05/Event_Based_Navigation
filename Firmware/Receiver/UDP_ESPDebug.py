import cv2
import socket
import numpy as np

dataprev = []


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
            
            #print(f"Receiving data ID: {data_id}, size: {data_size} bytes")
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
                    num1 = int.from_bytes(buffer[:4], byteorder='little')
                    num2 = int.from_bytes(buffer[4:8], byteorder='little')
                    num3 = int.from_bytes(buffer[8:12], byteorder='little')
                    num4 = int.from_bytes(buffer[12:], byteorder='little')

                    return data_id, data_size, num1, num2, num3, num4
                except Exception as e:
                    print(f"Error parsing status data: {e}")
                    return None

# Configuration
PC_IP = "0.0.0.0"       # Listen on all network interfaces
PC_PORT = 5005          # Port to receive video data
MAX_PACKET_SIZE = 65507  # Maximum UDP packet size

# Initialize socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((PC_IP, PC_PORT))

# Buffer and state for reassembling frame data
buffer = b""
frame_size = None

print(f"Listening for video stream on {PC_IP}:{PC_PORT}...")

try:
    while True:
        data = receiveData(0)
        data_id = data[0]
        data_size = data[1]


        if data_id == 0: #Regular Frames
            cv2.imshow("Video Stream", data[2])
            cv2.waitKey(1)
        elif data_id == 1: #Event Frames
            cv2.imshow("Video Stream", data[2])
            cv2.waitKey(1)
        elif data_id == 2: #From Pi
            print(f"Value 1: {data[2]}")
            print(f"Value 2: {data[3]}")
            print(f"Value 3: {data[4]}")
            print(f"Value 4: {data[5]}")
            
        elif data_id == 3: #From ESP32
            if data != dataprev:
                print(f"Value 1: {data[2]}")
                print(f"Value 2: {data[3]}")
                print(f"Value 3: {data[4]}")
                print(f"Value 4: {data[5]}")
                dataprev = data
        else:
            print("No Data Received")
            sleep_ms(200)


except KeyboardInterrupt:
    print("Stopped receiving video.")
finally:
    sock.close()
    cv2.destroyAllWindows()

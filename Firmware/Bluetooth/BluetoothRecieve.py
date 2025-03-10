
import socket

ESP32_MAC = "8C:4F:00:3D:53:64"  # Replace with your ESP32's Bluetooth MAC Address
PORT = 1  # RFCOMM Port

try:
    # Create Bluetooth RFCOMM socket
    sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    sock.connect((ESP32_MAC, PORT))
    print(f"Connected to ESP32 at {ESP32_MAC}")

    while True:
        data = sock.recv(1024)  # Receive up to 1024 bytes
        if data:
            print("Received:", data.decode("utf-8"))

except KeyboardInterrupt:
    print("\nDisconnected from ESP32.")

except Exception as e:
    print(f"Error: {e}")

finally:
    sock.close()
    print("Socket closed.")
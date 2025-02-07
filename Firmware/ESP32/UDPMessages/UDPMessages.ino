#include <WiFi.h>
#include <WiFiUdp.h>

// WiFi credentials
const char* ssid = "SARK";
const char* password = "samsamsam802";

// UDP configuration
WiFiUDP udp;
const char* udpAddress = "192.168.43.245";  // Sam's laptop
const int udpPort = 5005;  // Receiver port

void setup() {
    Serial.begin(115200);
    
    // Connect to WiFi
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
    }
    Serial.println("\nConnected to WiFi!");
}

void loop() {
    // Data ID (Leave as 3) / Number of bytes to send (4*4) / 32-bit ints to send
    int32_t numbers[6] = {3, 16, 30, 40, 50, 60};

    uint8_t buffer[24];  // 6 integers * 4 bytes each = 24 bytes
    memcpy(buffer, numbers, sizeof(numbers));  // Copy data into buffer

    // Send UDP message
    udp.beginPacket(udpAddress, udpPort);
    udp.write(buffer, sizeof(buffer));  // Send 24-byte buffer
    udp.endPacket();
    
    Serial.println("UDP packet sent!");
    delay(2000);  // Send every 2 seconds
}

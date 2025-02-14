#include "variables.h"

const MotorDriver motorDrivers[] = {
    {2, 25, 1, 0, 26, 3, 4},
    {7, 27, 6, 5, 23, 13, 12},
    {9, 19, 10, 11, 5, 8, 14}
};


PCF8575 pcf8575(0x20);  // PCF8575 object for I2C communication

IBusBM ibusRc;  // RC IBus communication object

HardwareSerial& ibusRcSerial = Serial2;  // Use Serial2 for IBus communication

const int ledPin = 2;  // Define the onboard LED pin

// previous direction pins for deciding if the motors need to stop
int prevRightDirection = 1;
int prevLeftDirection = 1;

// WiFi credentials
const char* ssid = "SARK";
const char* password = "samsamsam802";

// UDP configuration
WiFiUDP udp;
const char* udpAddress = "192.168.43.132";  // Doug's laptop
const int udpPort = 5005;  // Receiver port

// Task handles
TaskHandle_t motorControlTaskHandle = NULL;
TaskHandle_t ibusTaskHandle = NULL;
TaskHandle_t wifiComsTaskHandle = NULL;
TaskHandle_t wifiCheckConnectionTaskHandle = NULL;
TaskHandle_t updateStateTaskHandle = NULL;

//states
extern int32_t controlState = 0;
extern int32_t running = 0;
extern int32_t start = 0;

//debugging values
extern int32_t RC_connected = 0;

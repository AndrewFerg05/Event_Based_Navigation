#ifndef VARIABLES_H
#define VARIABLES_H

// Include necessary libraries
#include <PCF8575.h>  
#include <IBusBM.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#define NUM_MOTOR_DRIVERS 3
#define WIFI_TIMEOUT_MS 1000

struct MotorDriver {
    const int standby;
    const int pwma;
    const int ain1;
    const int ain2;
    const int pwmb;
    const int bin1;
    const int bin2;
};

extern const MotorDriver motorDrivers[];

// Declare external variables
extern PCF8575 pcf8575;
extern IBusBM ibusRc;

extern HardwareSerial& ibusRcSerial; // Serial port used for communication

extern const int ledPin;  // Onboard LED pin

extern int prevLeftDirection;
extern int prevRightDirection;

// WiFi credentials
extern const char* ssid;
extern const char* password;

// UDP configuration
extern WiFiUDP udp;
extern const char* udpAddress;
extern const int udpPort;

// Task handles
extern TaskHandle_t motorControlTaskHandle;
extern TaskHandle_t ibusTaskHandle;
extern TaskHandle_t wifiComsTaskHandle;
extern TaskHandle_t wifiCheckConnectionTaskHandle;
extern TaskHandle_t updateStateTaskHandle;

extern int32_t controlState;
extern int32_t running;
extern int32_t start;

// debugging varaibles
extern int32_t RC_connected;

#endif
#ifndef VARIABLES_H
#define VARIABLES_H

// Include necessary libraries
#include <PCF8575.h>  
#include <IBusBM.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "ICM_20948.h"

#define NUM_MOTOR_DRIVERS 3
#define WIFI_TIMEOUT_MS 1000
#define WHEEL_RADIUS 0.065
#define PPR 1133
#define WIRE_PORT Wire1 // desired Wire port.
#define AD0_VAL 1      // value of the last bit of the I2C address.


struct MotorDriver {
    const int standby;
    const int pwma;
    const int ain1;
    const int ain2;
    const int pwmb;
    const int bin1;
    const int bin2;
};

struct Position {
    float x;
    float y;
};

extern Position currentPos;

extern const MotorDriver motorDrivers[];

// Declare external variables
extern PCF8575 pcf8575;
extern IBusBM ibusRc;

extern HardwareSerial& ibusRcSerial; // Serial port used for communication

extern const int ledPin;  // Onboard LED pin

extern const int ENA_1;
extern const int ENB_R;

extern const int ENA_6;
extern const int ENB_L;

extern volatile int pos_1;
extern volatile int pos_2;
extern volatile int pos_3;
extern volatile int pos_4;
extern volatile int pos_5;
extern volatile int pos_6;

extern float displacement;
extern float heading;
extern float prevHeading;
extern float headingOffsets[200];

//IMU stuff 
extern ICM_20948_I2C imu;

extern float A_B[3];
extern float A_Ainv[3][3];
extern float M_B[3];
extern float M_Ainv[3][3];
extern float declination;
extern float p[3];

extern float headingOffset;



// Mutex for protecting access to the shared variables
extern SemaphoreHandle_t xPositionMutex;

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
extern TaskHandle_t displacementCalcTaskHandle;

extern int32_t controlState;
extern int32_t running;
extern int32_t start;

// debugging varaibles
extern int32_t RC_connected;

#endif
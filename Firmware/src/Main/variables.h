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
#define WINDOW_SIZE 25  // gives a delay of 500 ms which lines up with the required delay for the heading calc


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
extern float slip;
extern float heading;
extern float headingOffsets[200];
extern volatile float filteredHeading;
extern float displacementSum;
extern volatile int dataReady;

//IMU stuff 
extern ICM_20948_I2C imu;

extern float A_B[3];
extern float A_Ainv[3][3];
extern float M_B[3];
extern float M_Ainv[3][3];
extern float declination;
extern float p[3];

extern float headingOffset;

extern float headingBufferCos[WINDOW_SIZE];
extern float headingBufferSin[WINDOW_SIZE];
extern int headingBufferIdx;
extern float headingSumSin;
extern float headingSumCos;
extern float filteredHeading1;


// Mutex for protecting access to the shared variables
extern SemaphoreHandle_t xHeadingMutex;

extern int prevLeftDirection;
extern int prevRightDirection;
extern int pointTurn;

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
extern TaskHandle_t wifiStatesTaskHandle;
extern TaskHandle_t wifiCheckConnectionTaskHandle;
extern TaskHandle_t updateStateTaskHandle;
extern TaskHandle_t displacementCalcTaskHandle;
extern TaskHandle_t PIComsTaskHandle;
extern TaskHandle_t filterHeadingTaskHandle;
extern TaskHandle_t calcPathTaskHandle;
extern TaskHandle_t navigateTaskHandle;

// state variables

extern int32_t controlState;
extern int32_t runningState;
extern int32_t startState;
extern int stateChanged;
extern int desiredState;
extern int piState;
extern bool FLAG_PI_STARTED;
extern String receivedMessage;

extern int suspend;

// debugging varaibles
extern int32_t connectedRC;

#endif
#include "variables.h"

const MotorDriver motorDrivers[] = {
    {2, 25, 1, 0, 26, 3, 4},
    {7, 27, 6, 5, 23, 13, 12},
    {9, 19, 10, 11, 5, 8, 14}
};

Position currentPos = {0, 0};

PCF8575 pcf8575(0x20);  // PCF8575 object for I2C communication

IBusBM ibusRc;  // RC IBus communication object

HardwareSerial& ibusRcSerial = Serial2;  // Use Serial2 for IBus communication

const int ledPin = 2;  // Define the onboard LED pin

// motor encoder pins
const int ENA_1 = 18;
const int ENB_R = 4;

const int ENA_6 = 32;
const int ENB_L = 33;

volatile int pos_1 = 0;
volatile int pos_2 = 0;
volatile int pos_3 = 0;
volatile int pos_4 = 0;
volatile int pos_5 = 0;
volatile int pos_6 = 0;

float displacement = 0;
float heading = 0;
float prevHeading = 0;
float prevHeading2 = 0;

float headingOffsets[200] = {0};

//IMU stuff 
ICM_20948_I2C imu; // create an ICM_20948_I2C object imu;

//Accel scale: divide by 16604.0 to normalize. These corrections are quite small and probably can be ignored.
float A_B[3]
{-133.47 , 1.7 , -117.47};

float A_Ainv[3][3]
{ {  0.06271 , 0.00087 , 0.00021},
  {  0.00087 , 0.0617 , -0.00046},
  { 0.00021 , -0.00046 , 0.06051}
};

//Mag scale divide by 369.4 to normalize. These are significant corrections, especially the large offsets.
float M_B[3]
{ 98.42 , 5.53 , 388.54};

float M_Ainv[3][3]
{ {  4.01436 , -0.14836 , -0.05006},
  { -0.14836 , 3.78476 , 0.05516},
  { -0.05006 , 0.05516 , 3.72039}
};

// local magnetic declination in degrees
float declination = -1.06;

float p[3] = {1, 0, 0};  //X marking on sensor board points toward yaw = 0

float headingOffset = 0;

SemaphoreHandle_t xPositionMutex;

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
TaskHandle_t wifiStatesTaskHandle = NULL;
TaskHandle_t wifiCheckConnectionTaskHandle = NULL;
TaskHandle_t updateStateTaskHandle = NULL;
TaskHandle_t displacementCalcTaskHandle = NULL;
TaskHandle_t PIComsTaskHandle = NULL;

//states
int32_t controlState = 0;
int32_t runningState = 0;
int32_t startState = 0;

//debugging values
int32_t connectedRC = 0;

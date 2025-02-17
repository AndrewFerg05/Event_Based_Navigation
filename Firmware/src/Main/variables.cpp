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

//IMU stuff 
ICM_20948_I2C imu; // create an ICM_20948_I2C object imu;

//Accel scale: divide by 16604.0 to normalize. These corrections are quite small and probably can be ignored.
float A_B[3]
{-216.2 , -84.87 , -122.89};

float A_Ainv[3][3]
{ {  0.06059 , 0.00005 , -0.00005},
  {  0.00005 , 0.06056 , 0.00068},
  { -0.00005 , 0.00068 , 0.06087}
};

//Mag scale divide by 369.4 to normalize. These are significant corrections, especially the large offsets.
float M_B[3]
{ 76.42 , 0.86 , 431.03};

float M_Ainv[3][3]
{ {  5.50509 , -0.078 , -0.0108},
  { -0.078 , 5.15096 , 0.06757},
  { -0.0108 , 0.06757 , 5.43276}
};

// local magnetic declination in degrees
float declination = -1.06;

float p[3] = {1, 0, 0};  //X marking on sensor board points toward yaw = 0



SemaphoreHandle_t xPositionMutex;

// previous direction pins for deciding if the motors need to stop
int prevRightDirection = 1;
int prevLeftDirection = 1;

// WiFi credentials
const char* ssid = "PI-SARK";
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
TaskHandle_t displacementCalcTaskHandle = NULL;

//states
extern int32_t controlState = 0;
extern int32_t running = 0;
extern int32_t start = 0;

//debugging values
extern int32_t RC_connected = 0;

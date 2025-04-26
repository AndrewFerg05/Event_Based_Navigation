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

// left
const int ENA_6 = 32;
const int ENB_L = 33;

volatile int pos_1 = 0;
volatile int pos_2 = 0;
volatile int pos_3 = 0;
volatile int pos_4 = 0;
volatile int pos_5 = 0;
volatile int pos_6 = 0;

float displacement = 0;
float slip = 0.63;
float heading = 0;
float displacementSum = 0.0;

float headingOffsets[200] = {0};
volatile int dataReady = 1;


//IMU stuff 
ICM_20948_I2C imu; // create an ICM_20948_I2C object imu;

//Accel scale: divide by 16604.0 to normalize. These corrections are quite small and probably can be ignored.
float A_B[3]
{59.47, -36.27, -149.62};

float A_Ainv[3][3]
{ {   0.06194 , 0.00054 , 0.00015 },
  {  0.00054 , 0.05974 , 0.00012},
  { 0.00015 , 0.00012 , 0.06018}
};

//Mag scale divide by 369.4 to normalize. These are significant corrections, especially the large offsets.
/* prev curved
float M_B[3]
{77.65 , 105.24 , -101.55};

float M_Ainv[3][3]
{ { 5.26028 , -0.2175 , 0.2439},
  { -0.2175 , 4.40278 , 0.0435},
  { 0.2439 , 0.0435 , 5.31303}
};
*/

/* prev working in gym
float M_B[3]
{35.18 , 35.77 , -102.63};

float M_Ainv[3][3]
{ {  4.85801 , 0.06222 , -0.03067},
  { 0.06222 , 4.51616 , 0.0659},
  { -0.03067 , 0.0659 , 5.04455}
};
*/
/*
// first calibration in macance
float M_B[3]
{37.12, 27.76 , -105.89};

float M_Ainv[3][3]
{ {  3.93136 , 0.03118 , -0.035},
  { 0.03118 , 3.66419 , 0.03024},
  { -0.035 , 0.03024 , 4.02858}
};
*/


// second calibration in macance
float M_B[3]
{37.11 , 24.64 , -105.02};

float M_Ainv[3][3]
{ {3.75655 , 0.02971 , -0.02765},
  { 0.02971 , 3.4591 , 0.02072},
  { -0.02765 , 0.02072 , 3.81214}
};


/* prevoous kinda working 
float M_B[3]
{56.06 , 37.22 , -121.82};

float M_Ainv[3][3]
{ {  4.77272 , 0.09642 , -0.01464 },
  {  0.09642 , 4.43183 , 0.04999},
  { -0.01464 , 0.04999 , 4.89426}
};
*/

// local magnetic declination in degrees
float declination = -1.06;

float p[3] = {1, 0, 0};  //X marking on sensor board points toward yaw = 0

float headingOffset = 0;

float headingBufferCos[WINDOW_SIZE] = {0};
float headingBufferSin[WINDOW_SIZE] = {0};
int headingBufferIdx = 0;
float headingSumSin = 0;
float headingSumCos = 0;
float volatile filteredHeading = 0;
float filteredHeading1 = 0;

SemaphoreHandle_t xHeadingMutex;

// previous direction pins for deciding if the motors need to stop
int prevRightDirection = 1;
int prevLeftDirection = 1;
int pointTurn = 0;

// WiFi credentials
const char* ssid = "EBRover";
const char* password = "ebrover2025";

// UDP configuration
WiFiUDP udp;
const char* udpAddress = "192.168.43.245";  
const int udpPort = 5005;  // Receiver port

// Task handles
TaskHandle_t motorControlTaskHandle = NULL;
TaskHandle_t ibusTaskHandle = NULL;
TaskHandle_t wifiStatesTaskHandle = NULL;
TaskHandle_t wifiCheckConnectionTaskHandle = NULL;
TaskHandle_t updateStateTaskHandle = NULL;
TaskHandle_t displacementCalcTaskHandle = NULL;
TaskHandle_t PIComsTaskHandle = NULL;
TaskHandle_t filterHeadingTaskHandle = NULL;
TaskHandle_t calcPathTaskHandle = NULL;
TaskHandle_t navigateTaskHandle = NULL;

//states
int32_t controlState = -1;
int32_t runningState = -1;
int32_t startState = -1;
int stateChanged = 0;
// idle = 0, run =1, stop = 2
int desiredState = -1;
int piState = -1;
bool FLAG_PI_STARTED = false;
String receivedMessage = "";  // Variable to store the complete message

int suspend = 0;

//debugging values
int32_t connectedRC = 0;

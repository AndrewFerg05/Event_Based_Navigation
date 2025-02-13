#ifndef VARIABLES_H
#define VARIABLES_H

// Include necessary libraries
#include <PCF8575.h>  
#include <IBusBM.h>

#define NUM_MOTOR_DRIVERS 3

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

#endif
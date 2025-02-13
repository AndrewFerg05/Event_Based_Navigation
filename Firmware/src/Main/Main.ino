#include "motor_control.h"
#include "receiver.h"
#include "PCF8575.h"
#include <IBusBM.h>

// Keep global variables in main for now
IBusBM ibusRc;
HardwareSerial& ibusRcSerial = Serial2;
PCF8575 pcf8575(0x20);

const int ledPin = 2; // Onboard LED pin

const MotorDriver motorDrivers[] = {
    {2, 25, 1, 0, 26, 3, 4},
    {7, 27, 6, 5, 23, 13, 12},
    {9, 19, 10, 11, 5, 8, 14}
};

void setup() {
    Wire.begin(13, 14);  // 13 = SDA, 14 = SCL
    setupMotors();  // Call function from motor_control.cpp
    ibusRc.begin(ibusRcSerial);
    pinMode(ledPin, OUTPUT);
}

void loop() {
    int throttleVal = readChannel(3, -255, 255, 0);
    int steeringVal = readChannel(1, -255, 255, 0);

    controlMotors(throttleVal, steeringVal);  // Call function from motor_control.cpp
    
    if (throttleVal == 0) {
        digitalWrite(ledPin, HIGH);
    }
    delay(100);
    digitalWrite(ledPin, LOW);
}

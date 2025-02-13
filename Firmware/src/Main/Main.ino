#include "motor_control.h"
#include "variables.h"
#include "receiver.h"
#include "PCF8575.h"
#include <IBusBM.h>


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

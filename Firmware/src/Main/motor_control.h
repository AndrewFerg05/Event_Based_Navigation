#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include "PCF8575.h"
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

void setupMotors();
void controlMotors(int throttleVal, int steeringVal);
void move_motor(int forward, int rpm, int pwm, int direct_1, int direct_2, int &prevDirection);
void stop_motor(int pwm, int direct_1, int direct_2);

#endif
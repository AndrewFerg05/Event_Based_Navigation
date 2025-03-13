#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include "PCF8575.h"

void setupMotors();
void controlMotors(int throttleVal, int steeringVal);
void move_motor(int forward, int rpm, int pwm, int direct_2, int direct_1, int &prevDirection);
void stop_motor(int pwm, int direct_2, int direct_1);
void stopAllMotors();

#endif
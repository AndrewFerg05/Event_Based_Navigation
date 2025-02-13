#include "motor_control.h"
#include "variables.h"

void setupMotors() {
    pcf8575.begin();
    for (int i = 0; i < NUM_MOTOR_DRIVERS; i++) {
        pcf8575.pinMode(motorDrivers[i].standby, OUTPUT);
        pcf8575.digitalWrite(motorDrivers[i].standby, LOW);
        pinMode(motorDrivers[i].pwma, OUTPUT);
        pcf8575.pinMode(motorDrivers[i].ain1, OUTPUT);
        pcf8575.pinMode(motorDrivers[i].ain2, OUTPUT);
        pinMode(motorDrivers[i].pwmb, OUTPUT);
        pcf8575.pinMode(motorDrivers[i].bin1, OUTPUT);
        pcf8575.pinMode(motorDrivers[i].bin2, OUTPUT);
    }
    for (int i = 0; i < NUM_MOTOR_DRIVERS; i++) {
        pcf8575.digitalWrite(motorDrivers[i].standby, HIGH);
    }
}

void controlMotors(int throttleVal, int steeringVal) {
    int leftDirection = (throttleVal >= 0) ? 1 : 0;
    int rightDirection = (throttleVal >= 0) ? 1 : 0;
    
    int leftPWM = abs(throttleVal + steeringVal);
    int rightPWM = abs(throttleVal - steeringVal);

    int maxOutput = max(leftPWM, rightPWM);
    if (maxOutput > 255) {
        float scaleFactor = 255.0 / maxOutput;
        leftPWM *= scaleFactor;
        rightPWM *= scaleFactor;
    }

    move_motor(rightDirection, rightPWM, motorDrivers[0].pwma, motorDrivers[0].ain1, motorDrivers[0].ain2, rightDirection);
    move_motor(leftDirection, leftPWM, motorDrivers[2].pwmb, motorDrivers[2].bin1, motorDrivers[2].bin2, leftDirection);
}

void move_motor(int forward, int rpm, int pwm, int direct_1, int direct_2, int &prevDirection) {
    if (forward != prevDirection) {
        stop_motor(pwm, direct_1, direct_2);
        delay(50);
    }
    if (forward == 1) {
        pcf8575.digitalWrite(direct_1, LOW);
        pcf8575.digitalWrite(direct_2, HIGH);
        analogWrite(pwm, rpm);
    } else {
        pcf8575.digitalWrite(direct_1, HIGH);
        pcf8575.digitalWrite(direct_2, LOW);
        analogWrite(pwm, rpm);
    }
    prevDirection = forward;
}

void stop_motor(int pwm, int direct_1, int direct_2) {
    analogWrite(pwm, 0);
    pcf8575.digitalWrite(direct_1, LOW);
    pcf8575.digitalWrite(direct_2, LOW);
}

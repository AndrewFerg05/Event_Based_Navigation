#include "motor_control.h"
#include "variables.h"

void setupMotors() {
  // sets up all the motor pins as outputs
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

  // Initialize direction variables
  int leftDirection = 1;  // Default: Forward
  int rightDirection = 1; // Default: Forward

  // Define tolerance for throttle neutrality (e.g., ±15)
  const int throttleNeutralTolerance = 15;

  // Check if throttle is in neutral range
  // this is for a point turn
  if (abs(throttleVal) <= throttleNeutralTolerance) {
    // Steering determines direction when throttle is neutral
    leftDirection = (steeringVal >= 0) ? 1 : 0;  // Forward if steering is positive, reverse if negative
    rightDirection = (steeringVal >= 0) ? 0 : 1; // Reverse if steering is positive, forward if negative
  } else {
    // Throttle determines direction for both motors
    leftDirection = (throttleVal >= 0) ? 1 : 0;
    rightDirection = (throttleVal >= 0) ? 1 : 0;
  }
    
  // Skid-steer mixing logic
  int leftPWM = abs(throttleVal + steeringVal);  // Left motor value (absolute for PWM)
  int rightPWM = abs(throttleVal - steeringVal); // Right motor value (absolute for PWM)

  // Find the maximum absolute value of the left and right motor PWM
  int maxOutput = max(leftPWM, rightPWM);

  // If the combined left/right motor value exceeds 255, scale them down
  if (maxOutput > 255) {
    float scaleFactor = 255.0 / maxOutput;
    leftPWM = (int)(leftPWM * scaleFactor);  // Scale left motor value
    rightPWM = (int)(rightPWM * scaleFactor); // Scale right motor value
  }

  //set Right motors Driver0(A and B), Driver1 (A)
  //set left motors Driver1 (B), Driver2 (A and B)
  move_motor(rightDirection, rightPWM, motorDrivers[0].pwma, motorDrivers[0].ain1, motorDrivers[0].ain2, prevRightDirection);
  move_motor(rightDirection, rightPWM, motorDrivers[0].pwmb, motorDrivers[0].bin1, motorDrivers[0].bin2, prevRightDirection);
  move_motor(rightDirection, rightPWM, motorDrivers[1].pwma, motorDrivers[1].ain1, motorDrivers[1].ain2, prevRightDirection);

  move_motor(leftDirection, leftPWM,  motorDrivers[1].pwmb, motorDrivers[1].bin1, motorDrivers[1].bin2, prevLeftDirection);
  move_motor(leftDirection, leftPWM, motorDrivers[2].pwma, motorDrivers[2].ain1, motorDrivers[2].ain2, prevLeftDirection);
  move_motor(leftDirection, leftPWM, motorDrivers[2].pwmb, motorDrivers[2].bin1, motorDrivers[2].bin2, prevLeftDirection);
}

/*moves a motor given the following parameters
int forward, 1 is forward, 0 is backwards
int rpm, the pwm value for the motor (0-255)
int pwm, the pwm pin for that motor
int direct_1 and 2, the direction pins for that motor
direct_1 | direct_2 | result
   1     |    0     | forward
   0     |    1     | backward
   1     |    1     | brake
   0     |    0     | stop

int preDirection is the previous direction of the motor, need this to check if the motor needs to be stopped before changing direction
*/

void move_motor(int forward, int rpm, int pwm, int direct_2, int direct_1, int &prevDirection) {
    if (forward != prevDirection) {
        stop_motor(pwm, direct_1, direct_2);
    }
    if (forward == 1) {
        pcf8575.digitalWrite(direct_1, HIGH);
        pcf8575.digitalWrite(direct_2, LOW);
        analogWrite(pwm, rpm);
    } else {
        pcf8575.digitalWrite(direct_1, LOW);
        pcf8575.digitalWrite(direct_2, HIGH);
        analogWrite(pwm, rpm);
    }
    prevDirection = forward;
}

void stop_motor(int pwm, int direct_2, int direct_1) {
    analogWrite(pwm, 0);
    pcf8575.digitalWrite(direct_1, LOW);
    pcf8575.digitalWrite(direct_2, LOW);
}

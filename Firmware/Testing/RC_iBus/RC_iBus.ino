
#include "PCF8575.h"

#include <IBusBM.h>

IBusBM ibusRc;

HardwareSerial& ibusRcSerial = Serial2;

// Set i2c address
PCF8575 pcf8575(0x20);

const int ledPin = 2; // Onboard LED pin

// Define an array of structures to hold the pin assignments for each motor driver
struct MotorDriver {
  const int standby;
  const int pwma;
  const int ain1;
  const int ain2;
  const int pwmb;
  const int bin1;
  const int bin2;
};

const MotorDriver motorDrivers[] = {
  {2, 25, 1, 0, 26, 3, 4},
  {7, 27, 6, 5, 23, 13, 12},
  {9, 19, 10, 11, 5, 8, 14}
};

void setup() {
  
  Wire.begin(13, 14);  // 13 = SDA, 14 = SCL


    // Iterate over each motor driver and initialize its pins as outputs and set all ouputs LOW
  for (int i = 0; i < sizeof(motorDrivers) / sizeof(motorDrivers[0]); i++) {
    pcf8575.pinMode(motorDrivers[i].standby, OUTPUT);
    pcf8575.digitalWrite(motorDrivers[i].standby, LOW);

    pinMode(motorDrivers[i].pwma, OUTPUT);

    pcf8575.pinMode(motorDrivers[i].ain1, OUTPUT);
    pcf8575.digitalWrite(motorDrivers[i].ain1, LOW);

    pcf8575.pinMode(motorDrivers[i].ain2, OUTPUT);
    pcf8575.digitalWrite(motorDrivers[i].ain2, LOW);

    pinMode(motorDrivers[i].pwmb, OUTPUT);

    pcf8575.pinMode(motorDrivers[i].bin1, OUTPUT);
    pcf8575.digitalWrite(motorDrivers[i].bin1, LOW);

    pcf8575.pinMode(motorDrivers[i].bin2, OUTPUT);
    pcf8575.digitalWrite(motorDrivers[i].bin2, LOW);
  }

  pcf8575.begin();

  // sets the standby outputs to high to enable all the motor drivers.
  for (int i = 0; i < sizeof(motorDrivers) / sizeof(motorDrivers[0]); i++) {
    pcf8575.digitalWrite(motorDrivers[i].standby, HIGH);
  }

  // Initialize IBus communication on Serial2
  ibusRc.begin(ibusRcSerial);

  pinMode(ledPin, OUTPUT);

}

void loop() {
  // variable to hold the previous direction of the motors
  int prevLeftDirection = 1;  // Initial assumed direction (1 = forward)
  int prevRightDirection = 1;

  // Read PWM signals from receiver (might change how this is implimented later bu this works for now)
  int throttleVal = readChannel(3, -255, 255, 0); // Channel 2 (array index starts from 0)
  int steeringVal = readChannel(1, -255, 255, 0); // Channel 4

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
  //change this as needed
  move_motor(rightDirection, rightPWM, motorDrivers[0].pwma, motorDrivers[0].ain1, motorDrivers[0].ain2, prevRightDirection);
  move_motor(rightDirection, rightPWM, motorDrivers[0].pwmb, motorDrivers[0].bin1, motorDrivers[0].bin2, prevRightDirection);
  move_motor(rightDirection, rightPWM, motorDrivers[1].pwma, motorDrivers[1].ain1, motorDrivers[1].ain2, prevRightDirection);

  move_motor(leftDirection, leftPWM,  motorDrivers[1].pwmb, motorDrivers[1].bin1, motorDrivers[1].bin2, prevLeftDirection);
  move_motor(leftDirection, leftPWM, motorDrivers[2].pwma, motorDrivers[2].ain1, motorDrivers[2].ain2, prevLeftDirection);
  move_motor(leftDirection, leftPWM, motorDrivers[2].pwmb, motorDrivers[2].bin1, motorDrivers[2].bin2, prevLeftDirection);
    
  if (throttleVal == 0) {
    // no signal detected
    digitalWrite(ledPin, HIGH);
  }

  delay(100); // Adjust as needed
  digitalWrite(ledPin, LOW);
}

void move_motor(int forward, int rpm, int pwm, int direct_1, int direct_2, int &prevDirection) {
  if (forward != prevDirection) {
    stop_motor(pwm, direct_1, direct_2);
    delay(50);
  }

  // Update motor direction
  if (forward == 1) {
    pcf8575.digitalWrite(direct_1, LOW);
    pcf8575.digitalWrite(direct_2, HIGH);
    analogWrite(pwm, rpm);
  } else {
    pcf8575.digitalWrite(direct_1, HIGH);
    pcf8575.digitalWrite(direct_2, LOW);
    analogWrite(pwm, rpm);
  }

  prevDirection = forward;  // Modify original variable directly
}

void stop_motor(int pwm, int direct_1, int direct_2) {
  analogWrite(pwm, 0);        // Set PWM to 0 (no speed)
  pcf8575.digitalWrite(direct_1, LOW); // Both direction pins LOW
  pcf8575.digitalWrite(direct_2, LOW);
}

int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  uint16_t ch = ibusRc.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 992, 1995, minLimit, maxLimit);
}

bool redSwitch(byte channelInput, bool defaultValue) {
  int intDefaultValue = (defaultValue) ? 100 : 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}
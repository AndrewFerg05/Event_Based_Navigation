#include "motor_control.h"
#include "variables.h"
#include "receiver.h"
#include "PCF8575.h"
#include <IBusBM.h>


// Task handle for motor control
TaskHandle_t motorControlTaskHandle = NULL;
TaskHandle_t ibusTaskHandle = NULL;

void ibusTask(void *pvParameters) {
    for (;;) {
        ibusRc.loop();  // Process incoming data
        vTaskDelay(10 / portTICK_PERIOD_MS);  // Run every 10ms
    }
}

// Perform an action every 10 ticks.
void motorControlTask( void * pvParameters )
{
  for( ;; ) {

      // Read throttle and steering values from receiver
      int throttleVal = readChannel(3, -255, 255, 0);
      int steeringVal = readChannel(1, -255, 255, 0);

      // Control motors based on these values
      controlMotors(throttleVal, steeringVal);

      // LED control based on throttle value
      if (throttleVal == 0) {
          digitalWrite(ledPin, HIGH);  // Turn on LED if throttle is zero
      } else {
          digitalWrite(ledPin, LOW);   // Turn off LED otherwise
      }

      vTaskDelay(100/ portTICK_PERIOD_MS);
      
  }
}


void setup() {  
    Wire.begin(13, 14);  // 13 = SDA, 14 = SCL (I2C pins for ESP32)
    setupMotors();  // Initialize motors
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);
    ibusRc.begin(ibusRcSerial, IBUSBM_NOTIMER);
    

    // Create the motor control task
    xTaskCreate(
        motorControlTask,          // Task function
        "Motor Control Task",      // Task name
        5000,                      // Stack size (adjust as needed)
        NULL,                      // Task parameters
        1,                         // Task priority (1 is low)
        &motorControlTaskHandle    // Task handle
    );

    // Create the ibus coms task
    xTaskCreate(
        ibusTask,          // Task function
        "ibus task",      // Task name
        2048,                      // Stack size (adjust as needed)
        NULL,                      // Task parameters
        1,                         // Task priority (1 is low)
        &ibusTaskHandle    // Task handle
    );

}


void loop() {
    // loop() is not needed since the motor control is handled by the FreeRTOS task
    
}


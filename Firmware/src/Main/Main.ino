#include "motor_control.h"
#include "variables.h"
#include "receiver.h"
#include "PCF8575.h"
#include <IBusBM.h>


// Task handle for motor control
TaskHandle_t motorControlTaskHandle = NULL;
TaskHandle_t ibusTaskHandle = NULL;
TaskHandle_t wifiTaskHandle = NULL;

void wifiTask(void *pvParameters) {
  TickType_t xLastWakeTime_wifi;
  const TickType_t xFrequency_wifi = 100/ portTICK_PERIOD_MS;

    xLastWakeTime_wifi = xTaskGetTickCount ();
    for( ;; ) {

      // delays until exactly 100 ms since the last call
      vTaskDelayUntil( &xLastWakeTime_wifi, xFrequency_wifi );

      // put Wifi code here (will operate exactly once every 100 ms)
      // will probabily mess around will that timing but as a starting point
      // since this will eventually be multithreaded 
      // will probs need to figure out how to safely acess the values to send over wifi 
      // but ignore that for the minute (unless you already know how to do it)
           
    }

}


void ibusTask(void *pvParameters) {
    for (;;) {
        ibusRc.loop();  // Process incoming data
        vTaskDelay(10 / portTICK_PERIOD_MS);  // Run every 10ms
    }
}

// Perform an action every 10 ticks.
void motorControlTask( void * pvParameters )
{
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 100/ portTICK_PERIOD_MS;

    xLastWakeTime = xTaskGetTickCount ();

    for( ;; ) {
        // delays until exactly 100 ms since the last call
        vTaskDelayUntil( &xLastWakeTime, xFrequency );

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

        // Create the ibus coms task
    xTaskCreate(
        wifiTask,          // Task function
        "wifi task",      // Task name
        8192,                      // Stack size (adjust as needed)
        NULL,                      // Task parameters
        1,                         // Task priority (1 is low)
        &wifiTaskHandle    // Task handle
    );

}


void loop() {
    // loop() is not needed since the motor control is handled by the FreeRTOS task
    
}


#include "motor_control.h"
#include "variables.h"
#include "receiver.h"
#include "PCF8575.h"
#include <IBusBM.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// WiFi credentials
const char* ssid = "SARK";
const char* password = "samsamsam802";

// UDP configuration
WiFiUDP udp;
const char* udpAddress = "192.168.43.132";  // Doug's laptop
const int udpPort = 5005;  // Receiver port

// Task handle for motor control
TaskHandle_t motorControlTaskHandle = NULL;
TaskHandle_t ibusTaskHandle = NULL;
TaskHandle_t wifiComsTaskHandle = NULL;
TaskHandle_t wifiCheckConnectionTaskHandle = NULL;

void wifiComsTask(void *pvParameters) {
  TickType_t xLastWakeTime_wifi;
  // every 100 ms
  const TickType_t xFrequency_wifi = 1000/ portTICK_PERIOD_MS;

    xLastWakeTime_wifi = xTaskGetTickCount ();
    for( ;; ) {

      // delays until exactly 100 ms since the last call
      vTaskDelayUntil( &xLastWakeTime_wifi, xFrequency_wifi );

      // put Wifi code here (will operate exactly once every 100 ms)
      // will probabily mess around will that timing but as a starting point
      // since this will eventually be multithreaded 
      // will probs need to figure out how to safely acess the values to send over wifi 
      // but ignore that for the minute (unless you already know how to do it)
          // Data ID (Leave as 3) / Number of bytes to send (4*4) / 32-bit ints to send
      int32_t numbers[6] = {3, 16, 30, 40, 50, 60};

      uint8_t buffer[24];  // 6 integers * 4 bytes each = 24 bytes
      memcpy(buffer, numbers, sizeof(numbers));  // Copy data into buffer

      // Send UDP message
      udp.beginPacket(udpAddress, udpPort);
      udp.write(buffer, sizeof(buffer));  // Send 24-byte buffer
      udp.endPacket();
    
      Serial.println("UDP packet sent!");     
    }

}

// Task to check Wifi connection and reconnect if connection was lost
void wifiCheckConnectionTask(void *pvParameters) {
  for (;;) {
    if(WiFi.status() == WL_CONNECTED) {
      Serial.println("Connected");
      vTaskDelay(1000 / portTICK_PERIOD_MS);  // Run every 1 second

    } else {
      Serial.println("Connection Lost");
      WiFi.disconnect();
      WiFi.reconnect();

      vTaskDelay(10 / portTICK_PERIOD_MS);  // Run every 1 second
    }
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

    Serial.begin(115200);
    
    // Connect to WiFi
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
    }
    Serial.println("\nConnected to WiFi!");
    

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

        // Create the wifi coms task
    xTaskCreate(
        wifiCheckConnectionTask,          // Task function
        "wifi Check Connection Task",      // Task name
        8192,                      // Stack size (adjust as needed)
        NULL,                      // Task parameters
        1,                         // Task priority (1 is low)
        &wifiComsTaskHandle    // Task handle
    );

            // Create the wifi connection check task
    xTaskCreate(
        wifiComsTask,          // Task function
        "wifi Coms task",      // Task name
        8192,                      // Stack size (adjust as needed)
        NULL,                      // Task parameters
        1,                         // Task priority (1 is low)
        &wifiCheckConnectionTaskHandle    // Task handle
    );

}


void loop() {
    // loop() is not needed since the motor control is handled by the FreeRTOS task
    
}


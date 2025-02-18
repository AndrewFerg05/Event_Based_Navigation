#include "motor_control.h"
#include "variables.h"
#include "receiver.h"
#include "displacement.h"
#include "PCF8575.h"
#include <IBusBM.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>
#include <Filters.h>

// make this a debugging task, ie (show coms status, current sate etc)
void wifiComsTask(void *pvParameters) {
  TickType_t xLastWakeTime_wifi;
  // every 100 ms
  const TickType_t xFrequency_wifi = 1000/ portTICK_PERIOD_MS;

    xLastWakeTime_wifi = xTaskGetTickCount ();
    for( ;; ) {

      // delays until exactly 100 ms since the last call
      vTaskDelayUntil( &xLastWakeTime_wifi, xFrequency_wifi );
      /*

      // put Wifi code here (will operate exactly once every 100 ms)
      // will probabily mess around will that timing but as a starting point
      // since this will eventually be multithreaded 
      // will probs need to figure out how to safely acess the values to send over wifi 
      // but ignore that for the minute (unless you already know how to do it)
          // Data ID (Leave as 3) / Number of bytes to send (4*4) / 32-bit ints to send
      int32_t numbers[6] = {3, 16, RC_connected, controlState, running, start};

      uint8_t buffer[24];  // 6 integers * 4 bytes each = 24 bytes
      memcpy(buffer, numbers, sizeof(numbers));  // Copy data into buffer

      // Send UDP message
      udp.beginPacket(udpAddress, udpPort);
      udp.write(buffer, sizeof(buffer));  // Send 24-byte buffer
      udp.endPacket();
    
      //Serial.println("UDP packet sent!");     
      */
    }

}

// Task to check Wifi connection and reconnect if connection was lost
void wifiCheckConnectionTask(void *pvParameters) {
  for (;;) {
    /*
    if(WiFi.status() == WL_CONNECTED) {
      //Serial.println("Connected");
      vTaskDelay(1000 / portTICK_PERIOD_MS);  // Run every 1 second
      continue;
    }

    WiFi.begin(ssid, password);

    unsigned long startAttemptTime = millis();

    while(WiFi.status() != WL_CONNECTED && 
          millis() - startAttemptTime < WIFI_TIMEOUT_MS) {}

    if (WiFi.status() != WL_CONNECTED) {

      //Serial.println("Connection Failed");
      
      vTaskDelay(100 / portTICK_PERIOD_MS);  // Retry every 100 ms
    }*/ 
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Run every 1 second
  }
}


void ibusTask(void *pvParameters) {

  for (;;) {
      ibusRc.loop();  // Process incoming data
      vTaskDelay(10 / portTICK_PERIOD_MS);  // Run every 10ms
  }
}

// calculate the displacement of the Rover and transmit it though UDP 
void displacementCalcTask(void *pvParameters) {
  TickType_t xLastWakeTime_disp;
  // every 100 ms
  const TickType_t xFrequency_disp = 1000/ portTICK_PERIOD_MS;
    xLastWakeTime_disp = xTaskGetTickCount ();
    for( ;; ) {
      
      vTaskDelayUntil( &xLastWakeTime_disp, xFrequency_disp);

      pos_1 = pos_1 + 100;
      pos_6 = pos_6 + 100;

      // Take the mutex to safely access the shared position variables
      if (xSemaphoreTake(xPositionMutex, portMAX_DELAY) == pdTRUE) {

          // Safely read the shared position variables
          int posCopy1 = pos_1;
          int posCopy2 = pos_2;
          int posCopy3 = pos_3;
          int posCopy4 = pos_4;
          int posCopy5 = pos_5;
          int posCopy6 = pos_6;

          // reset position counters
          pos_1 = 0;
          pos_2 = 0;
          pos_3 = 0;
          pos_4 = 0;
          pos_5 = 0;
          pos_6 = 0;

          // Release the mutex after reading the data
          xSemaphoreGive(xPositionMutex);

          // when all the encoders are added in will need to average the values

          // 1,2,3 are the right side motors
          // 4,5,6 are the left side motors
          // convert to linear displacement


          displacement = ((((posCopy1 + posCopy6)/2.0) / PPR) * 2.0 * PI * WHEEL_RADIUS)*1000; // to mm
          Serial.print("displacement:  ");
          Serial.println(displacement);


          // read heading from IMU (use previous value for updating position)
          static float Axyz[3], Mxyz[3]; //centered and scaled accel/mag data

          if ( imu.dataReady() ) imu.getAGMT();

          get_scaled_IMU(Axyz, Mxyz);  //apply relative scale and offset to RAW data. UNITS are not important

          // reconcile mag and accel coordinate axes
          // Note: the illustration in the ICM_90248 data sheet implies that the magnetometer
          // Y and Z axes are inverted with respect to the accelerometer axes, verified to be correct (SJR).

          Mxyz[1] = -Mxyz[1]; //align magnetometer with accelerometer (reflect Y and Z)
          Mxyz[2] = -Mxyz[2];

          

          float headingTemp = get_heading(Axyz, Mxyz, p, declination);
          if (headingTemp < headingOffset) {
            heading = 360 - (headingOffset - headingTemp);
          } else {
            heading = headingTemp - headingOffset;
          }

          Serial.print("Heading: ");
          Serial.println(heading);
          Serial.print("Headingoffset: ");
          Serial.println(headingOffset);

          float headingRad = heading * (PI / 180.0);
          
          // update position 
          currentPos.x += (displacement * (float)cos(prevHeading));
          currentPos.y += (displacement * (float)sin(prevHeading));
          Serial.print("x:  ");
          Serial.println(currentPos.x);
          Serial.print("y:  ");
          Serial.println(currentPos.y);

          int32_t x = (int32_t)(currentPos.x);
          int32_t y = (int32_t)(currentPos.y);

          // transmit using UDP, or flag for another task to transmit the data
          /*
          int32_t numbers[6] = {3, 16, x, y, int32_t(prevHeading), (int32_t)(posCopy1)};

          uint8_t buffer[24];  // 6 integers * 4 bytes each = 24 bytes
          memcpy(buffer, numbers, sizeof(numbers));  // Copy data into buffer

          // Send UDP message
          udp.beginPacket(udpAddress, udpPort);
          udp.write(buffer, sizeof(buffer));  // Send 24-byte buffer
          udp.endPacket();
          */

          // 
          prevHeading = headingRad;    
      }
    }
}


void updateStateTask(void *pvParameters) {

  for (;;) {
      // code to read in and update the states 
      controlState = readChannel(6, 0, 1, 2);
      running = readChannel(7, 0, 1, 2);
      start = readChannel(8, 0, 1, 2);

      if (controlState == 2 || running == 2 || start == 2){
        RC_connected = 0;
      } else {
        RC_connected = 1;
      }


      vTaskDelay(100 / portTICK_PERIOD_MS);  // Run every 10ms
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

    // mutex to block access to position values
    xPositionMutex = xSemaphoreCreateMutex();

    // Attach interrupt for encoder A
    attachInterrupt(digitalPinToInterrupt(ENA_1), readEncoder1, RISING);
    attachInterrupt(digitalPinToInterrupt(ENA_6), readEncoder6, RISING);

    Serial.begin(115200);
    WIRE_PORT.begin(21, 22);
    WIRE_PORT.setClock(400000);
    imu.begin(WIRE_PORT, AD0_VAL);
    if (imu.status != ICM_20948_Stat_Ok) {
      //Serial.println(F("ICM_90248 not detected"));
    }
    /*
    // Connect to WiFi
    WiFi.begin(ssid, password);
    //Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        //Serial.print(".");
        delay(500);
    }
    */
    //Serial.println("\nConnected to WiFi!");

    float sum = 0;
    for (int i = 0; i < 200; i++) {
      static float Axyz[3], Mxyz[3]; //centered and scaled accel/mag data
      if ( imu.dataReady() ) imu.getAGMT();
      get_scaled_IMU(Axyz, Mxyz);  //apply relative scale and offset to RAW data. UNITS are not important
      Mxyz[1] = -Mxyz[1]; //align magnetometer with accelerometer (reflect Y and Z)
      Mxyz[2] = -Mxyz[2];

      headingOffsets[i] = get_heading(Axyz, Mxyz, p, declination);
      sum += headingOffsets[i];
      delay(100);
    }

    headingOffset = sum / 200.0;

    // Create the motor control task
    xTaskCreate(
        motorControlTask,          // Task function
        "Motor Control Task",      // Task name
        5000,                      // Stack size (adjust as needed)
        NULL,                      // Task parameters
        1,                         // Task priority (1 is low)
        &motorControlTaskHandle    // Task handle
    );

    // Create the RC ibus coms task
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

    // Create task to read RC state values from switches
    xTaskCreate(
        updateStateTask,          // Task function
        "update State Task",      // Task name
        1024,                      // Stack size (adjust as needed)
        NULL,                      // Task parameters
        1,                         // Task priority (1 is low)
        &updateStateTaskHandle   // Task handle
    );

    // Create the displacement estimation task
    xTaskCreate(
        displacementCalcTask,          // Task function
        "displacement estimate",      // Task name
        8192,                      // Stack size (adjust as needed)
        NULL,                      // Task parameters
        1,                         // Task priority (1 is low)
        &displacementCalcTaskHandle    // Task handle
    );

}


void loop() {
    // loop() is not needed since the motor control is handled by the FreeRTOS task
    
}


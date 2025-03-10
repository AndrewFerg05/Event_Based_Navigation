#include "motor_control.h"
#include "variables.h"
#include "receiver.h"
#include "displacement.h"
#include "PCF8575.h"
#include <IBusBM.h>
//#include <WiFi.h>
//#include <WiFiUdp.h>
#include <math.h>
#include "BluetoothSerial.h"


BluetoothSerial SerialBT;  // Bluetooth Serial object


void PIComsTask(void *pvParameters) {
  TickType_t xLastWakeTime_PI;
  // 100 ms 
  const TickType_t xFrequency_PI = 100/ portTICK_PERIOD_MS;
  xLastWakeTime_PI = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil( &xLastWakeTime_PI, xFrequency_PI);

    if (stateChanged == 1) {
      // transmit
    }

    while (Serial.available()) {
      char incomingChar = Serial.read();  // Read each character from the buffer
      
      if (incomingChar == '\n') {  // Check if end of message

        if (receivedMessage.startsWith("State: ")) {        // If message is about pi state
            piState = receivedMessage.substring(7).toInt();   // Extract integer state

            if (FLAG_PI_STARTED == false && piState == 0) {       // If Pi just booted and ready in Idle allow other code to start
              FLAG_PI_STARTED = true;
            }
        }
        receivedMessage = ""; // Clear the message buffer
      } else {
        // Append the character to the message string
        receivedMessage += incomingChar;
      }      
    }
    if (piState == desiredState) {
      stateChanged = 0;
    }
  }
}

/*

// make this a debugging task, ie (show coms status, current sate etc)
void wifiStatesTask(void *pvParameters) {
  for( ;; ) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for a notification
    
    // Data ID set to 4 for status bits / Number of bytes to send (4*4) / 32-bit ints to send
    int32_t numbers[6] = {4, 16, connectedRC, controlState, -1, -1};

    uint8_t buffer[24];  // 6 integers * 4 bytes each = 24 bytes
    memcpy(buffer, numbers, sizeof(numbers));  // Copy data into buffer

    // Send UDP message
    udp.beginPacket(udpAddress, udpPort);
    udp.write(buffer, sizeof(buffer));  // Send 24-byte buffer
    udp.endPacket();
    //Serial.println("UDP packet sent!");   
    
  }
} 
*/



// Task to check Wifi connection and reconnect if connection was lost
void wifiCheckConnectionTask(void *pvParameters) {
  for (;;) {
    /*
    
    if(WiFi.status() == WL_CONNECTED) {
      //Serial.println("Connected");
      vTaskDelay(1000 / portTICK_PERIOD_MS);  // Run every 1 second
      continue;
    }

    WiFi.begin(ssid);

    unsigned long startAttemptTime = millis();

    while(WiFi.status() != WL_CONNECTED && 
          millis() - startAttemptTime < WIFI_TIMEOUT_MS) {}

    if (WiFi.status() != WL_CONNECTED) {

      //Serial.println("Connection Failed");
      
      vTaskDelay(100 / portTICK_PERIOD_MS);  // Retry every 100 ms
    }
    */
    vTaskDelay(100 / portTICK_PERIOD_MS);  // Retry every 100 ms
    
  }
}



void ibusTask(void *pvParameters) {

  for (;;) {
      ibusRc.loop();  // Process incoming data
      vTaskDelay(10 / portTICK_PERIOD_MS);  // Run every 10ms
  }
}

//get heading values at maximum sample rate and filter using a moving average filter
void filterHeadingTask(void *pvParameters) {
  TickType_t xLastWakeTime_h;
  // 100 Hz (the max frequency of the mag)
  const TickType_t xFrequency_h = 20/ portTICK_PERIOD_MS;
    xLastWakeTime_h = xTaskGetTickCount();
    for( ;; ) {

      vTaskDelayUntil( &xLastWakeTime_h, xFrequency_h);

      // removes values from sum
      headingSumSin -= headingBufferSin[headingBufferIdx];
      headingSumCos -= headingBufferCos[headingBufferIdx];

      // read heading from IMU (use previous value for updating position)
      static float Axyz[3], Mxyz[3]; //centered and scaled accel/mag data

      if (imu.status != ICM_20948_Stat_Ok) {
        SerialBT.println("CL");
      }

      if (imu.dataReady()) {
        imu.getAGMT();
      } else {
        SerialBT.println("data not ready");
        imu.begin(WIRE_PORT, AD0_VAL);
      }

      get_scaled_IMU(Axyz, Mxyz);  //apply relative scale and offset to RAW data. UNITS are not important

      Mxyz[1] = -Mxyz[1]; //align magnetometer with accelerometer (reflect Y and Z)
      Mxyz[2] = -Mxyz[2];

      float headingTemp = get_heading(Axyz, Mxyz, p, declination);
      
      if (headingTemp < headingOffset) {
        heading = 360 - (headingOffset - headingTemp);
      } else {
        heading = headingTemp - headingOffset;
      }

      // convert to rads
      heading = heading * (PI / 180.0);

      // convert pol to cart
      headingBufferCos[headingBufferIdx] = cos(heading);
      headingBufferSin[headingBufferIdx] = sin(heading);

      // add to sums
      headingSumCos += headingBufferCos[headingBufferIdx];
      headingSumSin += headingBufferSin[headingBufferIdx];

      // update index
      headingBufferIdx = (headingBufferIdx + 1) % WINDOW_SIZE;

      // take average
      float averagedCos = headingSumCos / float(WINDOW_SIZE);
      float averagedSin = headingSumSin / float(WINDOW_SIZE);

      // cart to pol
      float copyFilteredHeading = atan2f(averagedSin, averagedCos);
      

      if (copyFilteredHeading < 0) {
        copyFilteredHeading += (2*PI);
      }


      // put mutex around this (window size is 20)
      // this gives a delay of 10 samples which means the displacement calculation
      // gets the heading from 1 time step before since it runs 10 times slower
      // this is intensional to due to the way the displacement is calculated
      if (xSemaphoreTake(xHeadingMutex, portMAX_DELAY)) {  // Lock mutex
    
        filteredHeading = copyFilteredHeading;
        xSemaphoreGive(xHeadingMutex);  // Unlock mutex
      }

      

    }

}

// calculate the displacement of the Rover and transmit it though UDP 
void displacementCalcTask(void *pvParameters) {
  TickType_t xLastWakeTime_disp;
  // every 500 ms
  const TickType_t xFrequency_disp = 100/ portTICK_PERIOD_MS;
    xLastWakeTime_disp = xTaskGetTickCount ();
    for( ;; ) {
      
      vTaskDelayUntil( &xLastWakeTime_disp, xFrequency_disp);
      // Take the mutex to safely access the shared position variables
      //if (xSemaphoreTake(xPositionMutex, portMAX_DELAY) == pdTRUE) {

      // Safely read the shared position variables
      int posCopy1 = pos_1;
      int posCopy2 = pos_2;
      int posCopy3 = pos_3;
      int posCopy4 = pos_4;
      int posCopy5 = pos_5;
      int posCopy6 = -pos_6;

      // reset position counters
      pos_1 = 0;
      pos_2 = 0;
      pos_3 = 0;
      pos_4 = 0;
      pos_5 = 0;
      pos_6 = 0;

      // Release the mutex after reading the data
      //xSemaphoreGive(xPositionMutex);

      // when all the encoders are added in will need to average the values
      // 1,2,3 are the right side motors
      // 4,5,6 are the left side motors
      // convert to linear displacement
      displacement = ((((posCopy1 + posCopy6)/2.0) / PPR) * 2.0 * PI * WHEEL_RADIUS) * 1000 * slip; // to mm

      float velocity = displacement/0.5;

      if (xSemaphoreTake(xHeadingMutex, portMAX_DELAY)) {  // Lock mutex
    
        filteredHeading1 = filteredHeading;
        xSemaphoreGive(xHeadingMutex);  // Unlock mutex
      }

      SerialBT.println(filteredHeading1);
           
      // update position 
      currentPos.x += (displacement * (float)cos(filteredHeading1));
      currentPos.y += (displacement * (float)sin(filteredHeading1));

      int32_t x = (int32_t)(currentPos.x);
      int32_t y = (int32_t)(currentPos.y);

      // transmit using UDP, or flag for another task to transmit the data
      /*
      int32_t numbers[6] = {3, 16, x, y, int32_t(filteredHeading1*10), int32_t(headingRawTemp)};

      uint8_t buffer[24];  // 6 integers * 4 bytes each = 24 bytes
      memcpy(buffer, numbers, sizeof(numbers));  // Copy data into buffer

      // Send UDP message
      udp.beginPacket(udpAddress, udpPort);
      udp.write(buffer, sizeof(buffer));  // Send 24-byte buffer
      udp.endPacket();    
      */  
    }
}
// update the states 
void updateStateTask(void *pvParameters) {
  static int32_t prevControlState = -1;
  static int32_t prevRunning = -1;
  static int32_t prevStart = -1;
  static int32_t prevConnectedRC = 0;

  for (;;) {
      // code to read in and update the states 
      controlState = readChannel(6, 0, 1, 2);
      runningState = readChannel(7, 0, 1, 2);
      startState = readChannel(8, 0, 1, 2);

      if (runningState != 2 && startState != 2 && runningState != -1 && runningState != -1) {
        int result = (startState << 1) | runningState;
        if (result == 4) {
          result = 2;
        }
        // add mutex
        desiredState = result;
      }

      if (desiredState == 2) {
        // reset position variables so I dont have to restart it everytime
        currentPos.x = 0;
        currentPos.y = 0;
      }

      if (controlState == 2 || runningState == 2 || startState == 2){
        desiredState = 2;
        connectedRC = 0;
      } else {
        connectedRC = 1;
      }

      // if one of the state varibales have changed send to the PI
      if (runningState != prevRunning || startState != prevStart) {
        stateChanged = 1;
      }

      // if the RC connected was lost or the control state was chnaged send to GUI
      if (controlState != prevControlState || connectedRC != prevConnectedRC) {
        //xTaskNotifyGive(wifiStatesTaskHandle);
      }
      

      prevControlState = controlState;
      prevRunning = runningState;
      prevStart = startState;
      prevConnectedRC = connectedRC;

      vTaskDelay(100 / portTICK_PERIOD_MS);  // Run every 10ms
  }
}


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
    xHeadingMutex = xSemaphoreCreateMutex();

    // Attach interrupt for encoder A
    pinMode(ENA_1, INPUT);
    pinMode(ENA_6, INPUT);
    pinMode(ENB_L, INPUT);
    pinMode(ENB_R, INPUT);

    attachInterrupt(digitalPinToInterrupt(ENA_1), readEncoder1, RISING);
    attachInterrupt(digitalPinToInterrupt(ENA_6), readEncoder6, RISING);

    //Serial.begin(115200);
    WIRE_PORT.begin(21, 22);
    WIRE_PORT.setClock(400000);
    imu.begin(WIRE_PORT, AD0_VAL);
    if (imu.status != ICM_20948_Stat_Ok) {
      //Serial.println(F("ICM_90248 not detected"));
    }

    /*
    // Connect to WiFi
    WiFi.begin(ssid);

    //int startTimeWC = millis();

    //attempt to connevt to wifi for 10 seconds
    while (WiFi.status() != WL_CONNECTED) {
        //Serial.print(".");
        delay(500);
    }
    */
    delay(2000);
    SerialBT.begin("ESP32_BT"); // Set Bluetooth device name
    

    digitalWrite(ledPin, HIGH);
    delay(1000);
    digitalWrite(ledPin, LOW);
    delay(1000);
    
    
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

    SerialBT.println(headingOffset);

    digitalWrite(ledPin, HIGH);
    delay(1000);
    digitalWrite(ledPin, LOW);
    delay(1000);

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
/*
    // send updated states UDP
    xTaskCreate(
        wifiStatesTask,          // Task function
        "send updated states UDP",      // Task name
        8192,                      // Stack size (adjust as needed)
        NULL,                      // Task parameters
        1,                         // Task priority (1 is low)
        &wifiStatesTaskHandle    // Task handle
    ); */

    // Create the wifi connection check task
    xTaskCreate(
        wifiCheckConnectionTask,          // Task function
        "wifi check connection task",      // Task name
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

    
    // Create the PI coms task to communicate states
    xTaskCreate(
        PIComsTask,          // Task function
        "PI ComsTask",      // Task name
        1024,                      // Stack size (adjust as needed)
        NULL,                      // Task parameters
        1,                         // Task priority (1 is low)
        &PIComsTaskHandle    // Task handle
    );
    

    // Create the filter heading task
    xTaskCreate(
        filterHeadingTask,          // Task function
        "filter Heading task",      // Task name
        2048,                      // Stack size (adjust as needed)
        NULL,                      // Task parameters
        1,                         // Task priority (1 is low)
        &filterHeadingTaskHandle    // Task handle
    );

    vTaskSuspend(PIComsTaskHandle);

}


void loop() {
    // loop() is not needed since the motor control is handled by the FreeRTOS task
    
}


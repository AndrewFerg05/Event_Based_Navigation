#include "motor_control.h"
#include "variables.h"
#include "receiver.h"
#include "displacement.h"
#include "PCF8575.h"
#include <IBusBM.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "stateChange.h"
#include <math.h>
//#include "BluetoothSerial.h"


//BluetoothSerial SerialBT;  // Bluetooth Serial object


void PIComsTask(void *pvParameters) {
  
  TickType_t xLastWakeTime_PI;
  // 100 ms 
  const TickType_t xFrequency_PI = 100/ portTICK_PERIOD_MS;
  xLastWakeTime_PI = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil( &xLastWakeTime_PI, xFrequency_PI);

    if (stateChanged == 1) {
      Serial.print("State: ");
      Serial.println(desiredState);
    }

    while (Serial.available()) {
      char incomingChar = Serial.read();  // Read each character from the buffer
      
      if (incomingChar == '\n') {  // Check if end of message

        if (receivedMessage.startsWith("State: ")) {        // If message is about pi state
            piState = receivedMessage.substring(7).toInt();   // Extract integer state

            if (FLAG_PI_STARTED == false && piState == 0) {       // If Pi just booted and ready in Idle allow other code to start
              FLAG_PI_STARTED = true;
            }
        } else if (receivedMessage.startsWith("Calib: ")) {
          int calib = receivedMessage.substring(7).toInt();
          digitalWrite(ledPin, calib);
        }
        receivedMessage = ""; // Clear the message buffer
      } else {
        // Append the character to the message string
        receivedMessage += incomingChar;
      }      
    }
    //SerialBT.print("Current State PI:  ");
    //SerialBT.println(piState);

    if (piState == desiredState) {
      stateChanged = 0;
    }
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
  // 50 Hz (half the max frequency of the mag)
  const TickType_t xFrequency_h = 20/ portTICK_PERIOD_MS;
    xLastWakeTime_h = xTaskGetTickCount();
    for( ;; ) {

      vTaskDelayUntil( &xLastWakeTime_h, xFrequency_h);

      // removes values from sum
      headingSumSin -= headingBufferSin[headingBufferIdx];
      headingSumCos -= headingBufferCos[headingBufferIdx];

      // read heading from IMU (use previous value for updating position)
      static float Axyz[3], Mxyz[3]; //centered and scaled accel/mag data

      if (imu.dataReady()) {
        imu.getAGMT();
        dataReady = 1;
      } else {
        // skip this iteration if data is not ready
        dataReady = 0;
        continue;
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

      if (heading < 0) {
        heading += (2*PI);
      }

      if (heading > (2*PI)) {
        heading -= (2*PI);
      }

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

      // put mutex around this (window size is 20)
      // this gives a delay of 10 samples which means the displacement calculation
      // gets the heading from 1 time step before since it runs 10 times slower
      // this is intensional to due to the way the displacement is calculated
      if (xSemaphoreTake(xHeadingMutex, portMAX_DELAY)) {  // Lock mutex
    
        filteredHeading = copyFilteredHeading;
        xSemaphoreGive(xHeadingMutex);  // Unlock mutex
      }

      if (suspend) {
        vTaskSuspend(NULL);
      }
    }

}

// calculate the displacement of the Rover and transmit it though UDP 
void displacementCalcTask(void *pvParameters) {
  TickType_t xLastWakeTime_disp;
  // every 500 ms
  const TickType_t xFrequency_disp = 250/ portTICK_PERIOD_MS;
    xLastWakeTime_disp = xTaskGetTickCount ();
    for( ;; ) {
      
      vTaskDelayUntil( &xLastWakeTime_disp, xFrequency_disp);
      // Take the mutex to safely access the shared position variables
      //if (xSemaphoreTake(xPositionMutex, portMAX_DELAY) == pdTRUE) {

      // Safely read the shared position variables
      int posCopy1 = pos_1;
      int posCopy6 = -pos_6;

      // reset position counters
      pos_1 = 0;
      pos_6 = 0;

      // Release the mutex after reading the data
      //xSemaphoreGive(xPositionMutex);
      // need to include all teh other motor encoders

      // when all the encoders are added in will need to average the values
      // 1,2,3 are the right side motors
      // 4,5,6 are the left side motors
      // convert to linear displacement
      displacement = ((((posCopy1 + posCopy6)/2.0) / PPR) * 2.0 * PI * WHEEL_RADIUS) * 100 * slip; // to cm

      if (pointTurn == 0) {
        displacement = displacement * 0.1;
      }

      displacementSum += displacement;

      float velocity = displacement/0.5;

      if (xSemaphoreTake(xHeadingMutex, portMAX_DELAY)) {  // Lock mutex
    
        filteredHeading1 = filteredHeading;
        xSemaphoreGive(xHeadingMutex);  // Unlock mutex
      }

               
      // update position 
      currentPos.x += (displacement * (float)sin(filteredHeading1));
      currentPos.y += (displacement * (float)cos(filteredHeading1));

      //String data = String(currentPos.x) + "," + String(currentPos.y) + "," + String(filteredHeading1) +"\n";
      //SerialBT.print(data);

      int32_t x = (int32_t)(currentPos.x);
      int32_t y = (int32_t)(currentPos.y);

      // transmit using UDP, or flag for another task to transmit the data
      
      int32_t numbers[7] = {3, 20, x, y, int32_t(filteredHeading1), int32_t(dataReady), int32_t(desiredState)};

      uint8_t buffer[28];  // 7 integers * 4 bytes each = 24 bytes
      memcpy(buffer, numbers, sizeof(numbers));  // Copy data into buffer

      // Send UDP message
      udp.beginPacket(udpAddress, udpPort);
      udp.write(buffer, sizeof(buffer));  // Send 24-byte buffer
      udp.endPacket();    
      
      if (suspend) {
        vTaskSuspend(NULL);
      }
    }
}
// update the states 
void updateStateTask(void *pvParameters) {
  static int32_t prevControlState = -1;
  static int32_t prevConnectedRC = 0;
  static int32_t prevDesiredState = -1;
  const int RUN = 1;
  const int IDLE = 0;
  const int STOP = 2;
  const int AUTO = 1;
  const int RC = 0;

  for (;;) {

      vTaskDelay(200 / portTICK_PERIOD_MS);
      // code to read in and update the states 
      controlState = readChannel(6, 0, 1, 2);
      runningState = readChannel(7, 0, 1, 2);
      startState = readChannel(8, 0, 1, 2);

      if (runningState != 2 && startState != 2 && runningState != -1 && runningState != -1) {
        int result = (startState << 1) | runningState;
        
        if (result == 1 && FLAG_PI_STARTED == false) {
          result = 0;
        }
        
        
        if (result == 3) {
          result = STOP;
        }
        // add mutex
        desiredState = result;
      }

      // if RC signal not deetcted then set the system to idle
      if (controlState == 2 || runningState == 2 || startState == 2){
        desiredState = IDLE;
      }

      // if the desired state has changed send to the PI
      if (desiredState != prevDesiredState) {
        stateChanged = 1;
      }

      // the state can only be in autonomous when in the run state
      if (desiredState == IDLE) {
        controlState = RC;
      }
      
      // if switched back to RC control then reenable the RC contorl task
      if (controlState != prevControlState) {
        if (controlState == RC && desiredState == RUN) {  
          if (eTaskGetState(motorControlTaskHandle) == eSuspended) {
            enableAllMotors();
            vTaskResume(motorControlTaskHandle);
          }
          // add code do disable the autonomous control tasks
        } else if (controlState == AUTO) {
          vTaskResume(calcPathTaskHandle);
        }
      }
    
      // check if state has chnaged from previous
      if (desiredState != prevDesiredState) {
        // if in idle the start PI coms, stop motors and reset position
        if (desiredState == IDLE) {
          if (eTaskGetState(PIComsTaskHandle) == eSuspended) {
            vTaskResume(PIComsTaskHandle);
          }
          suspend = 1;
          stopAllMotors();
          resetVariables();
          // if in run enable all mootors resume tasks
        } else if (desiredState == RUN) {
          suspend = 0;
          enableAllMotors();
          vTaskResume(filterHeadingTaskHandle);
          vTaskResume(displacementCalcTaskHandle);
          vTaskResume(motorControlTaskHandle);
        } else if (desiredState == STOP) {
          suspend = 1;
          vTaskDelay(20 / portTICK_PERIOD_MS);  // delay 20 ms
          stopAllMotors();
          
        }
      }

      //SerialBT.print("Current State ESP:  ");
      //SerialBT.println(desiredState);
      
      prevControlState = controlState;
      prevConnectedRC = connectedRC;
      prevDesiredState = desiredState;
      

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
            //digitalWrite(ledPin, HIGH);  // Turn on LED if throttle is zero
        } else {
            //digitalWrite(ledPin, LOW);   // Turn off LED otherwise
        }

        if (suspend == 1 || controlState == 1) {
          stopAllMotors();
          //SerialBT.print("Current Control State:  ");
          //SerialBT.println(controlState);
          vTaskSuspend(NULL);
        }          
    }
}

void calcPathTask(void *pvParameters) {
  for( ;; ) { 
    vTaskDelay(200 / portTICK_PERIOD_MS);  // give enough time for the RC tasks to finish

    // calculate path code
    //SerialBT.println("Path calculated");

    vTaskResume(navigateTaskHandle);
    vTaskSuspend(NULL);
  }
}

void navigateTask(void *pvParameters) {
  TickType_t xLastWakeTime;
  // every 100 ms
  const TickType_t xFrequency = 100/ portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount ();
    for( ;; ) { 
      vTaskDelayUntil( &xLastWakeTime, xFrequency);
      enableAllMotors();
      //SerialBT.println("Navigation action");

      // if sate switched back to RC then suspend this task
      if (controlState == 0) {
        stopAllMotors();
        if (desiredState == 1) {
          enableAllMotors();
          vTaskResume(motorControlTaskHandle);
        }
        vTaskSuspend(NULL);
      }

      if (desiredState == 2) {
        vTaskSuspend(NULL);
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

    Serial.begin(115200);
    WIRE_PORT.begin(21, 22);
    WIRE_PORT.setClock(400000);
    imu.begin(WIRE_PORT, AD0_VAL);
    if (imu.status != ICM_20948_Stat_Ok) {
      //Serial.println(F("ICM_90248 not detected"));
    }

    
    // Connect to WiFi
    WiFi.begin(ssid, password);

    //int startTimeWC = millis();

    //attempt to connevt to wifi if it fails this will block
    while (WiFi.status() != WL_CONNECTED) {
        //Serial.print(".");
        delay(500);
    }
    
    delay(2000);
    //SerialBT.begin("ESP32_Project"); // Set Bluetooth device name
    

    digitalWrite(ledPin, HIGH);
    delay(1000);
    digitalWrite(ledPin, LOW);
    delay(1000);

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

    //SerialBT.println(headingOffset);

    digitalWrite(ledPin, HIGH);
    delay(1000);
    digitalWrite(ledPin, LOW);
    delay(1000);
    
    createMotortask();
    createPipelinetasks();
    createNavigationTasks();
    createComsTasks();
}


void loop() {
    // loop() is not needed since the motor control is handled by the FreeRTOS task
    
}


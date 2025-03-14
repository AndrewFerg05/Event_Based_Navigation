#include "variables.h"
#include "stateChange.h"


void resetVariables() {
  currentPos.x = 0;
  currentPos.y = 0;

}

void createComsTasks() {

  // Create the RC ibus coms task
  xTaskCreate(
      ibusTask,          // Task function
      "ibus task",      // Task name
      2048,                      // Stack size (adjust as needed)
      NULL,                      // Task parameters
      1,                         // Task priority (1 is low)
      &ibusTaskHandle    // Task handle
  );

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
      2048,                      // Stack size (adjust as needed)
      NULL,                      // Task parameters
      1,                         // Task priority (1 is low)
      &updateStateTaskHandle   // Task handle
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
}

void createMotortask() {

  // Create the motor control task
  xTaskCreate(
      motorControlTask,          // Task function
      "Motor Control Task",      // Task name
      5000,                      // Stack size (adjust as needed)
      NULL,                      // Task parameters
      1,                         // Task priority (1 is low)
      &motorControlTaskHandle    // Task handle
  );

}

void createPipelinetasks() {
  // Create the displacement estimation task
  xTaskCreate(
      displacementCalcTask,          // Task function
      "displacement estimate",      // Task name
      8192,                      // Stack size (adjust as needed)
      NULL,                      // Task parameters
      1,                         // Task priority (1 is low)
      &displacementCalcTaskHandle    // Task handle
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
}
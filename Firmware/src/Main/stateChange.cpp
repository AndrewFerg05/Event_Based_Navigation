#include "variables.h"
#include "stateChange.h"


void resetVariables() {
  currentPos.x = 0;
  currentPos.y = 0;
  displacementSum = 0;

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
      4098,                      // Stack size (adjust as needed)
      NULL,                      // Task parameters
      1,                         // Task priority (1 is low)
      &updateStateTaskHandle   // Task handle
  );

  // Create the PI coms task to communicate states
  xTaskCreate(
      PIComsTask,          // Task function
      "PI ComsTask",      // Task name
      4096,                      // Stack size (adjust as needed)
      NULL,                      // Task parameters
      1,                         // Task priority (1 is low)
      &PIComsTaskHandle    // Task handle
  );

  vTaskSuspend(PIComsTaskHandle);

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

  vTaskSuspend(motorControlTaskHandle);

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

  vTaskSuspend(displacementCalcTaskHandle);

  // Create the filter heading task
  xTaskCreate(
      filterHeadingTask,          // Task function
      "filter Heading task",      // Task name
      8192,                      // Stack size (adjust as needed)
      NULL,                      // Task parameters
      1,                         // Task priority (1 is low)
      &filterHeadingTaskHandle    // Task handle
  ); 

  vTaskSuspend(filterHeadingTaskHandle);

}

void createNavigationTasks() {
  // Create the task to calculate the return path
  xTaskCreate(
      calcPathTask,          // Task function
      "Find Path",      // Task name
      4096,                      // Stack size (adjust as needed)
      NULL,                      // Task parameters
      1,                         // Task priority (1 is low)
      &calcPathTaskHandle    // Task handle
  );

  vTaskSuspend(calcPathTaskHandle);

  // Create the task to navigate along that return path
  xTaskCreate(
      navigateTask,          // Task function
      "navigate",      // Task name
      4096,                      // Stack size (adjust as needed)
      NULL,                      // Task parameters
      1,                         // Task priority (1 is low)
      &navigateTaskHandle    // Task handle
  );

  vTaskSuspend(navigateTaskHandle); 
}
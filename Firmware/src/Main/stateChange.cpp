#include "variables.h"
#include "stateChange.h"


void resetVariables() {
  currentPos.x = 0;
  currentPos.y = 0;

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
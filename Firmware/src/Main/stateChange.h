#ifndef STATECHANGE_H
#define STATECHANGE_H

extern void motorControlTask(void *pvParameters);
extern void displacementCalcTask(void *pvParameters);
extern void filterHeadingTask(void *pvParameters);
extern void PIComsTask(void *pvParameters);
extern void wifiCheckConnectionTask(void *pvParameters);
extern void ibusTask(void *pvParameters);
extern void updateStateTask(void *pvParameters);
extern void calcPathTask(void *pvParameters);
extern void navigateTask(void *pvParameters);
//extern void wifiStatesTask(void *pvParameters);

void resetVariables();

void createNavigationTasks();

void createMotortask();

void createPipelinetasks();

void createComsTasks();

#endif
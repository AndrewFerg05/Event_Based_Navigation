#ifndef STATECHANGE_H
#define STATECHANGE_H

extern void motorControlTask(void *pvParameters);
extern void displacementCalcTask(void *pvParameters);
extern void filterHeadingTask(void *pvParameters);

void resetVariables();

void createMotortask();

void createPipelinetasks();

#endif
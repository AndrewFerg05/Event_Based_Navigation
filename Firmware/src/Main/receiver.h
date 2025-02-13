#ifndef RECEIVER_H
#define RECEIVER.H

#include <Arduino.h>
#include <IBusBM.h>

int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue);
bool redSwitch(byte channelInput, bool defaultValue);

#endif
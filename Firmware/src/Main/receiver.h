#ifndef RECEIVER_H
#define RECEIVER_H

#include <Arduino.h>
#include <IBusBM.h>

int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue);
bool readSwitch(byte channelInput, bool defaultValue);

#endif
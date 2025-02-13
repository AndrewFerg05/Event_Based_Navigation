#include "receiver.h"

extern IBusBM ibusRc;

int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
    uint16_t ch = ibusRc.readChannel(channelInput);
    if (ch < 100) return defaultValue;
    return map(ch, 992, 1995, minLimit, maxLimit);
}

bool redSwitch(byte channelInput, bool defaultValue) {
    int intDefaultValue = (defaultValue) ? 100 : 0;
    int ch = readChannel(channelInput, 0, 100, intDefaultValue);
    return (ch > 50);
}
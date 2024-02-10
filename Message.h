#ifndef MESSAGE_H 
#define MESSAGE_H

#include <Arduino.h>
#include "Configure.h"
// #define _TIMERINTERRUPT_LOGLEVEL_     4
// #include "ESP32TimerInterrupt.h"

// Function prototypes
void initializeMsgTimer();
void startMsgTimerSequence(int onDuration, int offDuration, int count);
void ARDUINO_ISR_ATTR msgHandler();
void msgOn();
void msgOff();
void ARDUINO_ISR_ATTR toneHandler();
void startTone();
void stopTone();

#endif // MESSAGE_H

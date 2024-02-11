#ifndef MESSAGE_H 
#define MESSAGE_H

#include <Arduino.h>
#include "Configure.h"
#include "Logger.h"
// #define _TIMERINTERRUPT_LOGLEVEL_     4
// #include "ESP32TimerInterrupt.h"

// Function prototypes
void ARDUINO_ISR_ATTR beepSequenceHandler();
void ARDUINO_ISR_ATTR toneHandler();

void beeperInit();
void initBeeperTimer();
void startAsyncBeepSequence(int onDuration, int offDuration, int count);
void beepOn();
void beepOff();

void beep(bool isLongBeep);
void beepNumTimes(int count, bool isLongBeep);
void beginBeepSeq();

#endif // MESSAGE_H

#include "Beeper.h"

hw_timer_t * beepTimer = NULL;
hw_timer_t * toneTimer = NULL;
const int frequency = 600; // Frequency in Hz
const int toggleInterval = 1000000 / (2 * frequency); // Toggle interval in microseconds

volatile bool beepState = false;
volatile int beepCount = 0;
volatile int beepOnDuration = 0;
volatile int beepOffDuration = 0;
volatile bool isbeepSequenceActive = false;

void ARDUINO_ISR_ATTR beepSequenceHandler() {
  if (beepCount > 0) {
    beepState = !beepState; // Toggle the state
    beepState ? beepOn() : beepOff();
    // Set the timer for the next state duration in microseconds
    timerAlarmWrite(beepTimer, (beepState ? beepOffDuration : beepOnDuration) * 1000, true);
    if (!beepState) {
      beepCount--; // Only decrement on the transition to silence
    }
  } else {
    isbeepSequenceActive = false; // Reset the sequence state
    timerAlarmDisable(beepTimer); // Disable the timer if the sequence is complete
    digitalWrite(PIN_LED_MSG, LOW); // Ensure the LED is off
  }
}

void ARDUINO_ISR_ATTR toneHandler() {
  static bool toneState = false;
  toneState = !toneState;
  digitalWrite(PIN_SPEAKER, toneState ? HIGH : LOW);
}

void beeperInit() {
  pinMode(PIN_SPEAKER, OUTPUT);
  pinMode(PIN_LED_MSG, OUTPUT);
  digitalWrite(PIN_SPEAKER, LOW);
  digitalWrite(PIN_LED_MSG, LOW);
}

void initBeeperTimer() {
  beepTimer = timerBegin(1, 80, true); // Use timer 1, prescaler 80 (1MHz), count up
  timerAttachInterrupt(beepTimer, &beepSequenceHandler, true); // Attach the interrupt function

  toneTimer = timerBegin(2, 80, true); // Use a different timer number than the beep sequence timer
  timerAttachInterrupt(toneTimer, &toneHandler, true);
  timerAlarmWrite(toneTimer, toggleInterval, true);
}

void startAsyncBeepSequence(int onDuration, int offDuration, int count) {
  if (isbeepSequenceActive) {
    // log("Beep sequence already active");
    return;
  }
  isbeepSequenceActive = true;
  beepOnDuration = onDuration;
  beepOffDuration = offDuration;
  beepCount = count;
  timerAlarmWrite(beepTimer, beepOnDuration, true); // Set timer for the ON duration
  timerAlarmEnable(beepTimer); // Enable the timer
}

void beepOn() {
  #ifdef DEBUG
  digitalWrite(PIN_LED_MSG, HIGH);
  #endif
  #ifdef SPEAKER_ON
  startTone();
  #endif
}

void beepOff() {
  #ifdef DEBUG
  digitalWrite(PIN_LED_MSG, LOW);
  #endif
  #ifdef SPEAKER_ON
  stopTone();
  #endif
}

void startTone() {
  timerAlarmEnable(toneTimer);
}

void stopTone() {
  timerAlarmDisable(toneTimer);
  digitalWrite(PIN_SPEAKER, LOW); // Ensure the speaker is off
}

// Blocking beep functions
void beepNumTimes(int count, bool isLongBeep) {
  for (int i = 0; i < count; i++) {
    if (i > 0) {
      delay(250);
    }
    beep(isLongBeep);
  }
}

void beep(bool isLongBeep) {
  unsigned int beepDuration = isLongBeep ? 1000 : 250;
  #ifdef DEBUG
  digitalWrite(PIN_LED_MSG, HIGH);
  delay(beepDuration);
  digitalWrite(PIN_LED_MSG, LOW);
  #endif
  #ifdef SPEAKER_ON
  tone(PIN_SPEAKER, 600, beepDuration);
  delay(beepDuration);
  noTone(PIN_SPEAKER);
  #endif
}

void beginBeepSeq() {
  int i=0;
  for (i=0; i<10; i++) {
    tone(PIN_SPEAKER, 1600,1000);
    digitalWrite(PIN_LED_MSG, HIGH);
    delay(50);
    noTone(PIN_SPEAKER);
    digitalWrite(PIN_LED_MSG, LOW);
  }
  delay(1000);
}

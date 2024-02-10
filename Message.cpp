#include "Message.h"

hw_timer_t * msgTimer = NULL;
hw_timer_t * toneTimer = NULL;
const int frequency = 600; // Frequency in Hz
const int toggleInterval = 1000000 / (2 * frequency); // Toggle interval in microseconds

const byte PIN_SPEAKER = 14;
const byte PIN_LED_MSG = 4;

volatile bool msgState = false;
volatile int msgCount = 0;
volatile int msgOnDuration = 0;
volatile int msgOffDuration = 0;
volatile bool isMsgSequenceActive = false;

void initializeMsgTimer() {
  pinMode(PIN_SPEAKER, OUTPUT);
  pinMode(PIN_LED_MSG, OUTPUT);
  msgTimer = timerBegin(1, 80, true); // Use timer 1, prescaler 80 (1MHz), count up
  timerAttachInterrupt(msgTimer, &msgHandler, true); // Attach the interrupt function

  toneTimer = timerBegin(2, 80, true); // Use a different timer number than the msg sequence timer
  timerAttachInterrupt(toneTimer, &toneHandler, true);
  timerAlarmWrite(toneTimer, toggleInterval, true);
}

void startMsgTimerSequence(int onDuration, int offDuration, int count) {
  if (isMsgSequenceActive) {
    // Optionally, handle the error, e.g., by returning early or resetting the sequence
    Serial.println("Message sequence already active");
    return;
  }
  Serial.println("Starting message sequence");
  isMsgSequenceActive = true;
  msgOnDuration = onDuration;
  msgOffDuration = offDuration;
  msgCount = count;
  timerAlarmWrite(msgTimer, msgOnDuration, true); // Set timer for the ON duration
  timerAlarmEnable(msgTimer); // Enable the timer
}

void ARDUINO_ISR_ATTR msgHandler() {
  if (msgCount > 0) {
    msgState = !msgState; // Toggle the state
    msgState ? msgOn() : msgOff();
    // Set the timer for the next state duration in microseconds
    timerAlarmWrite(msgTimer, (msgState ? msgOffDuration : msgOnDuration) * 1000, true);
    if (!msgState) {
      msgCount--; // Only decrement on the transition to silence
    }
  } else {
    isMsgSequenceActive = false; // Reset the sequence state
    timerAlarmDisable(msgTimer); // Disable the timer if the sequence is complete
    digitalWrite(PIN_LED_MSG, LOW); // Ensure the LED is off
  }
}

void msgOn() {
  #ifdef DEBUG
  digitalWrite(PIN_LED_MSG, HIGH);
  #endif
  #ifdef SPEAKER_ON
  startTone();
  #endif
}

void msgOff() {
  #ifdef DEBUG
  digitalWrite(PIN_LED_MSG, LOW);
  #endif
  #ifdef SPEAKER_ON
  stopTone();
  #endif
}

void ARDUINO_ISR_ATTR toneHandler() {
  static bool toneState = false;
  toneState = !toneState;
  digitalWrite(PIN_SPEAKER, toneState ? HIGH : LOW);
}

void startTone() {
  timerAlarmEnable(toneTimer);
}

void stopTone() {
  timerAlarmDisable(toneTimer);
  digitalWrite(PIN_SPEAKER, LOW); // Ensure the speaker is off
}

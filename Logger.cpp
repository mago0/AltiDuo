#include "Logger.h"

void log(const String &message) {
  #ifdef DEBUG
  Serial.println(message);
  #endif
}

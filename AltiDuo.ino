#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "Adafruit_BMP3XX.h"
typedef Adafruit_BMP3XX Barometer;
#include "Configure.h"
#include "Message.h"

const double SEA_LEVEL_PRESSURE = 1013.7;

Barometer bmp;

hw_timer_t * timer0 = NULL;

// Double buffer for altitude
volatile int altitudeBufferIndex = 0;
const int altitudeBufferSize = 2;
int altitudeBuffer[altitudeBufferSize] = {0, 0};

volatile SemaphoreHandle_t altitudeSemaphore;

//ground level altitude
long initialAltitude;
//current altitude
long currAltitude;
//Apogee altitude
long apogeeAltitude;
long mainAltitude;
long liftoffAltitude;
long lastAltitude;
long landingAltitude;

//Our drogue has been ejected i.e: apogee has been detected
boolean mainHasFired =false;
boolean apogeeSaved = false;
//nbr of measures to do so that we are sure that apogee has been reached 
unsigned long apogeeMeasures;
unsigned long mainDeployAltitude;

const byte PIN_I2C_SDA            = 23;
const byte PIN_I2C_SCL            = 22;
const byte PIN_SD_CS              = 16; 
const byte PIN_ALTITUDE           = 32;
const byte PIN_APOGEE             = 33;
const byte PIN_MAIN               = 15;
const byte PIN_APOGEE_CONTINUITY  = 12;
const byte PIN_MAIN_CONTINUITY    = 27;
const byte PIN_SPEAKER            = 14;
const byte PIN_LED_OK             = 17;
const byte PIN_LED_MSG            = 4;

struct KalmanFilter {
  float f_1 = 1.00000; // cast as float
  float x;
  float x_last;
  float p;
  float p_last;
  float k;
  float q;
  float r;
  float x_temp;
  float p_temp;
};

KalmanFilter kalman;

enum RocketState {
  PAD,
  ASCENDING,
  DESCENDING,
  LANDED
};

RocketState currentState;

void setup() {
  KalmanInit();
  
  #ifdef DEBUG
  Serial.begin(115200);
  #endif
  
  //Presure Sensor Initialisation
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  if (!bmp.begin_I2C()) {
    #ifdef DEBUG
    Serial.println("Barometer Failed");
    #endif
    while(1);
  }

  //our drogue has not been fired
  mainHasFired=false;
  
  //Initialise the output pin
  pinMode(PIN_APOGEE, OUTPUT);
  pinMode(PIN_MAIN, OUTPUT);
  pinMode(PIN_SPEAKER, OUTPUT);
  pinMode(PIN_LED_OK, OUTPUT);
  pinMode(PIN_LED_MSG, OUTPUT);
  
  pinMode(PIN_ALTITUDE, INPUT);
  
  pinMode(PIN_APOGEE_CONTINUITY, INPUT);
  pinMode(PIN_MAIN_CONTINUITY, INPUT);
  //Make sure that the output are turned off
  digitalWrite(PIN_APOGEE, LOW);
  digitalWrite(PIN_MAIN, LOW);
  digitalWrite(PIN_SPEAKER, LOW);
  
  //initialisation give the version of the altimeter
  //One long beep per major number and One short beep per minor revision
  //For example version 1.2 would be one long beep and 2 short beep
  beepAltiVersion(MAJOR_VERSION, MINOR_VERSION);
 
  // Read in the previous altitude
  int previousApogee = readPreviousApogee();
  if (previousApogee > 0 ) {

    previousApogee *= FEET_IN_METER;
    for (int repeat = 0; repeat < 2; repeat++) {   
      beepAltitude(previousApogee);

      delay(1000);
   }
  }
  
  //number of measures to do to detect Apogee
  apogeeMeasures = 10;
  
  initialAltitude = getInitialAltitude();

  // Read the deployment altitude from the potentiometer
  mainDeployAltitude = readDeploymentAltitude();
  
  lastAltitude = 0; 
  liftoffAltitude = 20;
  landingAltitude = 10;

  altitudeSemaphore = xSemaphoreCreateBinary();

  timer0 = timerBegin(0, 80, true); // Use timer 0, prescaler 80 (1MHz), count up
  timerAttachInterrupt(timer0, &timerHandlerAltitude, true); // Attach the interrupt function
  timerAlarmWrite(timer0, 20000, true); // Set timer for 50Hz frequency (1 second / 50 = 0.02 seconds or 20,000 microseconds)

  initializeMsgTimer();

  currentState = PAD; 
  digitalWrite(PIN_LED_OK, HIGH);
}

void loop() {
  if (takeAltitudeSemaphore()) {
    currAltitude = KalmanCalc(readAltitudeBuffer()) - initialAltitude;
  }
  Serial.println("Current Altitude: " + String(currAltitude) + " meters");

  switch (currentState) {
  case PAD:
    if (currAltitude < liftoffAltitude) {
      #ifdef CONTINUITY_CHECK
      continuityCheck();      
      #endif
    } else {
      currentState = ASCENDING;
    }
    break;
  
  case ASCENDING:
    if (currAltitude < lastAltitude) {
      apogeeMeasures = apogeeMeasures - 1;
      if (apogeeMeasures == 0) {
        //fire drogue
        currentState = DESCENDING;
        digitalWrite(PIN_APOGEE, HIGH);
        delay (2000);
        digitalWrite(PIN_APOGEE, LOW);
        apogeeAltitude = currAltitude;
      }  
    } else {
      lastAltitude = currAltitude;
      apogeeMeasures = 10;
    } 
    break;

  case DESCENDING:
    if (mainHasFired == false && currAltitude < mainDeployAltitude) {
      digitalWrite(PIN_MAIN, HIGH);
      delay(2000);
      mainHasFired = true;
      mainAltitude = currAltitude;
      digitalWrite(PIN_MAIN, LOW);
    }

    if (mainHasFired == true && currAltitude < landingAltitude) {
      currentState = LANDED;
      timerEnd(timer0);
      if(apogeeSaved == false ) {
        writePreviousApogee(apogeeAltitude); 
        apogeeSaved = true;
      }

      beginBeepSeq();

      beepAltitude(apogeeAltitude * FEET_IN_METER);

      delay(1000);
    }
    break;

  default:
    break;
  }
}

void ARDUINO_ISR_ATTR timerHandlerAltitude() {
  writeAltitudeBuffer(bmp.readAltitude(SEA_LEVEL_PRESSURE));
  switchAltitudeBuffer();
  xSemaphoreGiveFromISR(altitudeSemaphore, NULL);
}

enum ContinuityCheckType {
  CHECK_APOGEE = 0,
  CHECK_MAIN = 1
};

ContinuityCheckType nextContinuityCheckType = CHECK_APOGEE;
unsigned long lastContinuityCheckTime = 0;

void continuityCheck() {
  byte msgCount = 1;
  byte pin = PIN_APOGEE_CONTINUITY;
  if (nextContinuityCheckType == CHECK_MAIN) {
    msgCount = 2;
    pin = PIN_MAIN_CONTINUITY;
  }
  if ((millis() - lastContinuityCheckTime) > 2000) {
    int continuityValue = digitalRead(pin);
    if (continuityValue == 0) {
      startMsgTimerSequence(1000, 250, msgCount);
    } else {
      startMsgTimerSequence(250, 250, msgCount);
    }
    // Toggle the check type for the next call
    nextContinuityCheckType = (nextContinuityCheckType == CHECK_APOGEE) ? CHECK_MAIN : CHECK_APOGEE;
    lastContinuityCheckTime = millis();
  }
}

int getInitialAltitude() {
  bmp.readAltitude(SEA_LEVEL_PRESSURE); // throw away the first reading
  // let's do some dummy altitude reading
  // to initialise the Kalman filter
  for (int i=0; i<50; i++){
    KalmanCalc(bmp.readAltitude(SEA_LEVEL_PRESSURE));
  }
  //let's read the lauch site altitude
  long sum = 0;
  long curr = 0;
  for (int i=0; i<10; i++){
    curr = KalmanCalc(bmp.readAltitude(SEA_LEVEL_PRESSURE));
    sum += curr;
    delay(50); }
  int initialAltitude = (sum / 10.0);
  #ifdef DEBUG
  Serial.println("Initial Altitude: " + String(initialAltitude) + " meters");
  #endif
  return initialAltitude;
}

int readDeploymentAltitude() {
  int altitudeSetting = analogRead(PIN_ALTITUDE); // Read the analog value from the potentiometer
  int deploymentAltitude = map(altitudeSetting, 0, 1023, 100, 500); // Map the value to a range of 100 to 500 meters
  #ifdef DEBUG
  Serial.println("Main Deployment Altitude set to: " + String(deploymentAltitude) + " meters");
  #endif
  return deploymentAltitude;
}

void longBeepRepeat( int digit ) {
  int i;
  for( i=0; i< digit; i++ ) {
    if( i > 0 ) {
      delay(250);
    }
    longBeep();
  }
}

void shortBeepRepeat( int digit ) {
  int i;
  for( i=0; i< digit; i++ ) {
    if( i > 0 ) {
      delay(250);
    }
    shortBeep();
  }
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
void longBeep() {
  #ifdef DEBUG
  digitalWrite(PIN_LED_MSG, HIGH);
  delay(1000);
  digitalWrite(PIN_LED_MSG, LOW);
  #else
  digitalWrite(PIN_LED_MSG, HIGH);
  tone(PIN_SPEAKER, 600,1000);
  delay(1000);
  digitalWrite(PIN_LED_MSG, LOW);
  noTone(PIN_SPEAKER);
  #endif
}
void shortBeep() {
  #ifdef DEBUG
  digitalWrite(PIN_LED_MSG, HIGH);
  delay(250);
  digitalWrite(PIN_LED_MSG, LOW);
  #else
  tone(PIN_SPEAKER, 600,250);
  digitalWrite(PIN_LED_MSG, HIGH);
  delay(250);
  noTone(PIN_SPEAKER);
  digitalWrite(PIN_LED_MSG, LOW);
  #endif
}

//================================================================
// Kalman functions in your code
//================================================================

//Call KalmanInit() once.  

//KalmanInit() - Call before any iterations of KalmanCalc()
void KalmanInit() {
  kalman.q = 4.0001;  //filter parameters, you can play around with them
  kalman.r = .20001;  // but these values appear to be fairly optimal

  kalman.x = 0;
  kalman.p = 0;
  kalman.x_temp = 0;
  kalman.p_temp = 0;
  
  kalman.x_last = 0;
  kalman.p_last = 0;
}

//KalmanCalc() - Calculates new Kalman values from float value "altitude"
// This will be the ASL altitude during the flight, and the AGL altitude during dumps
float KalmanCalc (float altitude) {
  //Predict kalman.x_temp, kalman.p_temp
  kalman.x_temp = kalman.x_last;
  kalman.p_temp = kalman.p_last + kalman.r;
  
  //Update kalman values
  kalman.k = (kalman.f_1 / (kalman.p_temp + kalman.q)) * kalman.p_temp;
  kalman.x = kalman.x_temp + (kalman.k * (altitude - kalman.x_temp));
  kalman.p = (kalman.f_1 - kalman.k) * kalman.p_temp;
  
  //Save this state for next time
  kalman.x_last = kalman.x;
  kalman.p_last = kalman.p;
  
  //Assign current Kalman filtered altitude to working variables
  //KAlt = kalman.x; //FLOAT Kalman-filtered altitude value
  return kalman.x;
}  

void beepAltiVersion (int majorNbr, int minorNbr) {
  #ifdef DEBUG
  Serial.println("Major Version:");
  Serial.println( majorNbr);
  Serial.println("Minor Version:");
  Serial.println( minorNbr);
  #endif

  longBeepRepeat(majorNbr);
  digitalWrite(PIN_LED_MSG, HIGH);
  delay(250);
  shortBeepRepeat(minorNbr);
  digitalWrite(PIN_LED_MSG, LOW);
  delay(1000);
}

/*************************************************************************

 The following is some Code written by Leo Nutz and modified so that it works
  Output the maximum achieved altitude (apogee) via flashing LED / Buzzer

 *************************************************************************/
void beepAltitude(uint32_t value) {
  char Apogee_String[5];                      // Create an array with a buffer of 5 digits

  ultoa(value, Apogee_String, 10);            // Convert unsigned long to string array, radix 10 for decimal
  uint8_t length = strlen(Apogee_String);     // Get string length

  delay(3000);                                // Pause for 3 seconds

  for(uint8_t i = 0; i < length; i++ ) {
    delay(1000);                              // Pause 1 second for every digit output

    uint8_t digit = (Apogee_String[i] - '0'); // Convert ASCI to actual numerical digit
    if ( digit == 0 ) {
      digit = 10;
    }
    if ( digit == 0 ) {
      longBeepRepeat(1);
    } else {
      shortBeepRepeat(digit);
    }
  }
}

#define CONFIG_START 32

struct ConfigStruct {
  char app[7];
  int  majorVersion;
  int  minorVersion;
  int  altitude;
  int  cksum;  
};

int readPreviousApogee() {
  
  ConfigStruct config;
  
  int i;
  for( i = 0; i < sizeof(config); i++ ) {
    *((char*)&config + i) = EEPROM.read(CONFIG_START + i);
  }
  // Verify:
  if ( strcmp( "AltDuo", config.app ) != 0 ) {
    return -1;
  }
  if ( config.majorVersion != MAJOR_VERSION ) {
    return -1;
  }
  if (config.minorVersion != MINOR_VERSION ) {
    return -1;
  }
  if ( config.cksum != 0xBA ) {
    return -1;
  }
  
  return config.altitude;
}

int writePreviousApogee( int altitude ) {
 ConfigStruct config;
 config.app[0] = 'A';
 config.app[1] = 'l';
 config.app[2] = 't';
 config.app[3] = 'D';
 config.app[4] = 'u';
 config.app[5] = 'o';
 config.app[6] = 0;
 config.majorVersion = MAJOR_VERSION;
 config.minorVersion = MINOR_VERSION;
 config.cksum = 0xBA;
 config.altitude = altitude;
 
 int i;
 for( i=0; i<sizeof(config); i++ ) {
   EEPROM.write(CONFIG_START+i, *((char*)&config + i));
 }
}

// Double buffer for altitude

void switchAltitudeBuffer() {
  altitudeBufferIndex = (altitudeBufferIndex + 1) % altitudeBufferSize;
}

int readAltitudeBuffer() {
  int readIndex = (altitudeBufferIndex + 1) % altitudeBufferSize;
  return altitudeBuffer[readIndex];
}

void writeAltitudeBuffer(int altitude) {
  altitudeBuffer[altitudeBufferIndex] = altitude;
}

bool takeAltitudeSemaphore() {
  return xSemaphoreTake(altitudeSemaphore, 0) == pdTRUE;
}

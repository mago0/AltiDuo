#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "Adafruit_BMP3XX.h"
typedef Adafruit_BMP3XX Barometer;
#include "Configure.h"
#include "Logger.h"
#include "Beeper.h"


Barometer bmp;

// Timer for hardware interrupts
hw_timer_t * timer0 = NULL;

// Semaphore for altitude buffer synchronization
volatile SemaphoreHandle_t altitudeSemaphore;

// Double buffer for altitude management
volatile int altitudeBufferIndex = 0;
const int altitudeBufferSize = 2;
int altitudeBuffer[altitudeBufferSize] = {0, 0};

// Altitude related variables
long initialAltitude; // Ground level altitude
long currAltitude;    // Current altitude
long lastAltitude;    // Last recorded altitude
long apogeeAltitude;  // Maximum altitude reached (apogee)
long mainAltitude;    // Altitude for main parachute deployment

// Apogee detection and parachute deployment variables
boolean mainHasFired = false; // Indicates if the main parachute has been deployed
unsigned long apogeeFireTime = 0; // Time when the apogee parachute was deployed
unsigned long mainFireTime = 0;   // Time when the main parachute was deployed
boolean apogeeSaved = false;      // Indicates if apogee data has been saved
unsigned long mainDeployAltitude; // Altitude for main parachute deployment

// Kalman filter for altitude smoothing
struct KalmanFilter {
  float f_1 = 1.00000; // Prediction factor
  float x;             // Filtered value
  float x_last;        // Previous filtered value
  float p;             // Estimation error covariance
  float p_last;        // Previous estimation error covariance
  float k;             // Kalman gain
  float q;             // Process noise covariance
  float r;             // Measurement noise covariance
  float x_temp;        // Temporary filtered value
  float p_temp;        // Temporary estimation error covariance
};

KalmanFilter kalman;

// State of the rocket
enum RocketState {
  PAD,
  ASCENDING,
  DESCENDING,
  LANDED
};
RocketState currentState;

// Type of continuity check to perform
enum ContinuityCheckType {
  CHECK_APOGEE = 0,
  CHECK_MAIN = 1
};
ContinuityCheckType nextContinuityCheckType = CHECK_APOGEE;

void setup() {
  #ifdef DEBUG
  Serial.begin(115200);
  #endif

  // Kalman Filter Initialization
  KalmanInit();

  // Pressure Sensor Initialization
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  if (!bmp.begin_I2C()) {
    log("Barometer Failed");
    while(1);
  }

  // Initialize the output pins and ensure they are turned off
  pinMode(PIN_APOGEE, OUTPUT);
  digitalWrite(PIN_APOGEE, LOW);
  pinMode(PIN_MAIN, OUTPUT);
  digitalWrite(PIN_MAIN, LOW);
  pinMode(PIN_LED_MSG, OUTPUT);

  // Initialize the input pins
  pinMode(PIN_ALTITUDE, INPUT);
  pinMode(PIN_APOGEE_CONTINUITY, INPUT);
  pinMode(PIN_MAIN_CONTINUITY, INPUT);

  // Beeper Initialization
  beeperInit();
  beepAltiVersion(MAJOR_VERSION, MINOR_VERSION); // Indicate the version of the altimeter with beeps

  // Read and beep the previous apogee altitude
  int previousApogee = readPreviousApogee();
  if (previousApogee > 0) {
    previousApogee *= FEET_IN_METER;
    for (int repeat = 0; repeat < 2; repeat++) {
      beepAltitude(previousApogee);
      delay(1000);
    }
  }

  // Get initial altitude and set deployment altitudes
  initialAltitude = getInitialAltitude();
  mainDeployAltitude = readDeploymentAltitude();
  lastAltitude = 0;

  // Semaphore and Timer Initialization for altitude management
  altitudeSemaphore = xSemaphoreCreateBinary();
  timer0 = timerBegin(0, 80, true); // Use timer 0, prescaler 80 (1MHz), count up
  timerAttachInterrupt(timer0, &timerHandlerAltitude, true); // Attach the interrupt function
  timerAlarmWrite(timer0, 20000, true); // Set timer for 50Hz frequency (20,000 microseconds)

  // Initialize the beeper timer for asynchronous beeping
  initBeeperTimer();

  // Set initial state of the rocket and indicate readiness
  currentState = PAD;
  digitalWrite(PIN_LED_OK, HIGH);
}

byte remainingApogeeMeasures = APOGEE_MEASURES;

void loop() {
  if (takeAltitudeSemaphore()) {
    currAltitude = KalmanCalc(readAltitudeBuffer()) - initialAltitude;
  }
  if (currAltitude != lastAltitude) {
    log("Alt: " + String(currAltitude) + " meters");
  }

  switch (currentState) {
  case PAD:
    if (currAltitude < LIFTOFF_ALTITUDE) {
      #ifdef CONTINUITY_CHECK
      continuityCheck();      
      #endif
    } else {
      currentState = ASCENDING;
    }
    break;
  
  case ASCENDING:
    if (currAltitude < lastAltitude) {
      remainingApogeeMeasures--;
      if (remainingApogeeMeasures == 0) {
        //fire drogue
        currentState = DESCENDING;
        digitalWrite(PIN_APOGEE, HIGH);
        apogeeFireTime = millis(); // Record the time when the apogee parachute was deployed
        apogeeAltitude = currAltitude;
      }  
    } else {
      remainingApogeeMeasures = APOGEE_MEASURES;
    }
    break;

  case DESCENDING:
    if (mainFireTime == 0 && currAltitude < mainDeployAltitude) {
      digitalWrite(PIN_MAIN, HIGH);
      mainFireTime = millis(); // Record the time when the main parachute was deployed
      mainAltitude = currAltitude;
      digitalWrite(PIN_MAIN, LOW);
    }

    if (mainFireTime > 0 && currAltitude < LANDING_ALTITUDE) {
      currentState = LANDED;
      timerEnd(timer0);
      if(!apogeeSaved) {
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

  lastAltitude = currAltitude;

  // Turn off the pyros after the fire duration to prevent potential battery drain or damage.
  if (apogeeFireTime > 0 && (millis() - apogeeFireTime) > PYRO_FIRE_DURATION) {
    digitalWrite(PIN_APOGEE, LOW);
    apogeeFireTime = 0;
  }
  if (mainFireTime > 0 && (millis() - mainFireTime) > PYRO_FIRE_DURATION) {
    digitalWrite(PIN_MAIN, LOW);
    mainFireTime = 0;
  }
}

void ARDUINO_ISR_ATTR timerHandlerAltitude() {
  writeAltitudeBuffer(bmp.readAltitude(SEA_LEVEL_PRESSURE));
  switchAltitudeBuffer();
  xSemaphoreGiveFromISR(altitudeSemaphore, NULL);
}

unsigned long lastContinuityCheckTime = 0;

// non-blocking
void continuityCheck() {
  byte msgCount = 1;
  byte pin = PIN_APOGEE_CONTINUITY;
  if (nextContinuityCheckType == CHECK_MAIN) {
    msgCount = 2;
    pin = PIN_MAIN_CONTINUITY;
  }
  // check continuity every 6 seconds
  if ((millis() - lastContinuityCheckTime) > CONTINUITY_CHECK_INTERVAL * 1000) {
    int continuityValue = digitalRead(pin);
    if (continuityValue == 0) {
      startAsyncBeepSequence(1500, 250, msgCount);
    } else {
      startAsyncBeepSequence(200, 150, msgCount);
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
  log("Initial Altitude: " + String(initialAltitude) + " meters");
  return initialAltitude;
}

int readDeploymentAltitude() {
  int altitudeSetting = analogRead(PIN_ALTITUDE); // Read the analog value from the potentiometer
  int deploymentAltitude = map(altitudeSetting, 0, 1023, 100, 500); // Map the value to a range of 100 to 500 meters
  log("Main Deployment Altitude set to: " + String(deploymentAltitude) + " meters");
  return deploymentAltitude;
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
  log("Major Version: " + String(majorNbr));
  log("Minor Version: " + String(minorNbr));

  beepNumTimes(majorNbr, true);
  digitalWrite(PIN_LED_MSG, HIGH);
  delay(250);
  beepNumTimes(minorNbr, false);
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
      beepNumTimes(1, true);
    } else {
      beepNumTimes(digit, false);
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

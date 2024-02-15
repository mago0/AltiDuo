#include <Arduino.h>
#include <Wire.h>
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

unsigned long liftoffTime = 0;

// Apogee detection and parachute deployment variables
boolean mainHasFired = false; // Indicates if the main parachute has been deployed
unsigned long apogeeTime = 0; // Time when the apogee parachute was deployed
unsigned long mainTime = 0;   // Time when the main parachute was deployed
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

// Simulation variables
long simulatedAltitude = 0;
bool isAscending = true;

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
  pinMode(PIN_TEST, INPUT_PULLUP);

  // Beeper Initialization
  beeperInit();
  beepAltiVersion(MAJOR_VERSION, MINOR_VERSION); // Indicate the version of the altimeter with beeps

  // Get initial altitude and set deployment altitudes
  initialAltitude = getInitialAltitude();
  mainDeployAltitude = readDeploymentAltitude();
  lastAltitude = 0;

  // Semaphore and Timer Initialization for altitude management
  altitudeSemaphore = xSemaphoreCreateBinary();
  timer0 = timerBegin(0, 80, true); // Use timer 0, prescaler 80 (1MHz), count up
  timerAttachInterrupt(timer0, &timerHandlerAltitude, true); // Attach the interrupt function
  timerAlarmWrite(timer0, 20000, true); // Set timer for 50Hz frequency (20,000 microseconds)
  timerAlarmEnable(timer0);

  // Initialize the beeper timer for asynchronous beeping
  initBeeperTimer();

  // Set initial state of the rocket and indicate readiness
  currentState = PAD;
  digitalWrite(PIN_LED_OK, HIGH);
}

byte remainingApogeeMeasures = APOGEE_MEASURES;

void loop() {
  // refresh altitude on 50Hz timer
  if (takeAltitudeSemaphore()) {
    if (digitalRead(PIN_TEST) == LOW) {
      currAltitude = simulateFlightData();
    } else {
      currAltitude = KalmanCalc(bmp.readAltitude(SEA_LEVEL_PRESSURE)) - initialAltitude;
    }
  }
  if (currAltitude != lastAltitude && currAltitude % 100 == 0) {
    log("Altitude: " + String(currAltitude) + " meters");
  }

  switch (currentState) {
  case PAD:
    if (currAltitude < LIFTOFF_ALTITUDE) {
      #ifdef CONTINUITY_CHECK
      continuityCheck();      
      #endif
    } else {
      log("ASCENDING");
      currentState = ASCENDING;
    }
    break;
  
  case ASCENDING:
    if (currAltitude < lastAltitude) {
      remainingApogeeMeasures = remainingApogeeMeasures - 1;
      if (remainingApogeeMeasures == 0) {
        //fire drogue
        log("APOGEE");
        currentState = DESCENDING;
        digitalWrite(PIN_APOGEE, HIGH);
        apogeeTime = millis() - liftoffTime;
        apogeeAltitude = currAltitude;
      }  
    }
    break;

  case DESCENDING:
    if (mainTime == 0 && currAltitude < mainDeployAltitude) {
      log("MAIN");
      digitalWrite(PIN_MAIN, HIGH);
      mainTime = millis() - liftoffTime;
      mainAltitude = currAltitude;
    }

    if (mainTime > 0 && currAltitude < LANDING_ALTITUDE) {
      log("LANDED");
      currentState = LANDED;
      timerEnd(timer0);
      // if(!apogeeSaved) {
      //   writePreviousApogee(apogeeAltitude);
      //   apogeeSaved = true;
      // }
    }
    break;

  case LANDED:
    report();
    delay(1000);

  default:
    break;
  }

  lastAltitude = currAltitude;

  // Turn off the pyros after the fire duration to prevent potential battery drain or damage.
  if (digitalRead(PIN_APOGEE) == HIGH && (millis() - apogeeTime) > PYRO_FIRE_DURATION) {
    digitalWrite(PIN_APOGEE, LOW);
  }
  if (digitalRead(PIN_MAIN) == HIGH && (millis() - mainTime) > PYRO_FIRE_DURATION) {
    digitalWrite(PIN_MAIN, LOW);
  }
}

void ARDUINO_ISR_ATTR timerHandlerAltitude() {
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

void beepAltitude(uint32_t altitude) {
  char altitudeStr[10]; // Buffer to hold the altitude as a string
  itoa(altitude, altitudeStr, 10); // Convert the altitude to a string

  for (int i = 0; altitudeStr[i] != '\0'; i++) {
    int digit = altitudeStr[i] - '0'; // Convert character to digit

    // one long beep for a zero
    if (digit > 0) {
      beepNumTimes(digit, false);
    } else {
      beepNumTimes(1, true);
    }

    // If it's not the last digit, wait a bit longer to separate the digits
    if (altitudeStr[i + 1] != '\0') {
      delay(1000); // Wait for 1 second between digits
    }
  }

  // Wait a bit longer after the full number has been blinked
  delay(2000);
}

bool takeAltitudeSemaphore() {
  return xSemaphoreTake(altitudeSemaphore, 0) == pdTRUE;
}

// Function to simulate flight data
long simulateFlightData() {
  if (isAscending) {
    // Increment altitude until we reach the apogee
    simulatedAltitude += SIMULATION_ASCENT_INCREMENT;
    if (simulatedAltitude >= SIMULATION_APOGEE) {
      // We've reached apogee, start descending
      isAscending = false;
    }
  } else {
    // Decrement altitude until we reach the ground
    simulatedAltitude -= SIMULATION_DESCENT_INCREMENT;
    if (simulatedAltitude <= 0) {
      // We've reached the ground, stop decrementing
      simulatedAltitude = 0;
    }
  }

  return simulatedAltitude;
}

void report() {
  log("Flight Statistics:");
  log("Apogee Altitude: " + String(apogeeAltitude * FEET_IN_METER) + " feet");
  log("Main Parachute Deployment Altitude: " + String(mainAltitude * FEET_IN_METER) + " feet");
  log("Time to Apogee: " + String(apogeeTime / 1000) + " seconds");
  log("Time to Main Parachute Deployment: " + String(mainTime / 1000) + " seconds");

  beepAltitude(apogeeAltitude * FEET_IN_METER);
}
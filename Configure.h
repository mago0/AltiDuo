#ifndef CONFIGURE_H
#define CONFIGURE_H

#define _TIMERINTERRUPT_LOGLEVEL_ 4

#define MAJOR_VERSION 2
#define MINOR_VERSION 1

// Turn on Serial Console
#define DEBUG
#undef SPEAKER_ON

#define CONTINUITY_CHECK
#define CONTINUITY_CHECK_INTERVAL 5

// Use Metric (meters) for altitude reporting
///#define METRIC_UNIT
#undef METRIC_UNIT

#ifdef METRIC_UNIT
#define FEET_IN_METER 1.0
#else
#define FEET_IN_METER 3.28084
#endif

#define PYRO_FIRE_DURATION 5000

#define SEA_LEVEL_PRESSURE 1013.7

#define PIN_I2C_SDA           23
#define PIN_I2C_SCL           22
#define PIN_SD_CS             16 
#define PIN_ALTITUDE          32
#define PIN_APOGEE            33
#define PIN_MAIN              15
#define PIN_APOGEE_CONTINUITY 12
#define PIN_MAIN_CONTINUITY   27
#define PIN_SPEAKER           14
#define PIN_LED_OK            17
#define PIN_LED_MSG           4
#define PIN_TEST              13

#define APOGEE_MEASURES       10 // number of measurements for apogee detection
#define LIFTOFF_ALTITUDE      20 // threshold for liftoff
#define LANDING_ALTITUDE      20 // threshold for landing

#endif
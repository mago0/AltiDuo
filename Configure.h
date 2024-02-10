#ifndef CONFIGURE_H
#define CONFIGURE_H

#define _TIMERINTERRUPT_LOGLEVEL_ 4

#define MAJOR_VERSION 2
#define MINOR_VERSION 1

// Turn on Serial Console
#define DEBUG
#define SPEAKER_ON

#define CONTINUITY_CHECK

// Use Metric (meters) for altitude reporting
///#define METRIC_UNIT
#undef METRIC_UNIT

#ifdef METRIC_UNIT
#define FEET_IN_METER 1.0
#else
#define FEET_IN_METER 3.28084
#endif

#endif
#ifndef GPS_FEATURE_H
#define GPS_FEATURE_H

#include <Arduino.h>

/* Debug mode enable flag */
#define DEBUG_MODE_ENABLED   true

/* Global system timestamp (milliseconds) */
extern unsigned long systemCurrentTimeMs;

/* Variable to store the final generated Google Maps URL */
extern String googleMapUrl;

/* Function declarations */
String sendData(String atCommand, const int timeoutMs, boolean enableDebug);
String parseGpsToMapLink(String gpsResponseRaw, boolean enableDebugPrint);

#endif
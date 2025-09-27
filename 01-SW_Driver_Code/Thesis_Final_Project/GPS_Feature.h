#ifndef GPS_FEATURE_H
#define GPS_FEATURE_H

#include "Generic_API.h"

/* Global system timestamp (milliseconds) */
extern unsigned long systemCurrentTimeMs;

/* Variable to store the final generated Google Maps URL */
extern String googleMapUrl;

/* Function declarations */
String parseGpsToMapLink(String gpsResponseRaw, boolean enableDebugPrint);
/****************************************************************************************
 *  Function Name    : requestGpsLocation
 *  Description      : Handles periodic retrieval of GNSS (GPS) location information from 
 *                     the GSM/GNSS module and forwards serial data for debugging. 
 *                     The function performs three main tasks:
 *                       1. Checks if the configured interval has elapsed since the last 
 *                          GPS request. If yes, it sends the "AT+CGNSSINFO" command to 
 *                          the module and parses the response into a Google Maps link.
 *                       2. Forwards any user-entered AT commands from the USB Serial 
 *                          monitor to the GSM serial port (transparent passthrough).
 *                       3. Forwards any responses or notifications from the GSM serial 
 *                          port back to the USB Serial monitor for debugging and testing.
 *
 *  Input Parameters :
 *    - systemCurrentTimeMs : (Reference) Stores the last GPS request timestamp in 
 *                            milliseconds. Updated internally when a new request is sent.
 *    - intervalMs          : Interval time (in milliseconds) that must elapse before 
 *                            sending the next GPS request.
 *    - debug               : Boolean flag; when true, both the raw GPS response and the 
 *                            parsed map link are printed to the Serial Monitor for analysis.
 *
 *  Return Value     :
 *    None
 *
 ****************************************************************************************/
void requestGpsLocation(unsigned long &systemCurrentTimeMs, unsigned long intervalMs, bool debug);
#endif
#include "GPS_Feature.h"

/* Variable to store the final generated Google Maps URL */
String googleMapUrl = "";



/****************************************************************************************
 *  Function Name    : parseGpsToMapLink
 *  Description      : Parses the raw +CGNSSINFO response from GSM/GNSS module, extracts
 *                     latitude and longitude fields, converts them into human-readable 
 *                     DMS (Degrees, Minutes, Seconds) format, and generates a Google Maps
 *                     link with decimal degree coordinates.
 *
 *  Input Parameters :
 *    gpsResponseRaw   - (String) Raw response string containing "+CGNSSINFO" data.
 *    enableDebugPrint - (boolean) If true, debug messages and parsed output are printed.
 *
 *  Return Value     :
 *    (String) A Google Maps URL containing the parsed latitude and longitude.
 ****************************************************************************************/
String parseGpsToMapLink(String gpsResponseRaw, boolean enableDebugPrint) 
{
    /* Find the index of "+CGNSSINFO:" substring in the response */
    int gnsInfoStartIndex = gpsResponseRaw.indexOf("+CGNSSINFO:");
    
    /* If "+CGNSSINFO:" is not found, print error message and return empty string */
    if (gnsInfoStartIndex == -1) {
        if (enableDebugPrint) {
            Serial.println("Error: +CGNSSINFO not found in response");
        }
        return googleMapUrl;
    }

    /* Extract substring starting right after "+CGNSSINFO:" */
    String gnsInfoData = gpsResponseRaw.substring(gnsInfoStartIndex + 11);
    
    /* Remove leading and trailing whitespaces from extracted data */
    gnsInfoData.trim();

    /* Array to store positions (indices) of commas in the data string */
    int commaIndices[20];  
    
    /* Counter for the number of commas found */
    int commaFoundCount = 0;

    /* Loop through the string to record indices of commas */
    for (int i = 0; i < gnsInfoData.length() && commaFoundCount < 20; i++) {
        if (gnsInfoData[i] == ',') {
            commaIndices[commaFoundCount++] = i;
        }
    }

    /* Validate that we have enough fields: 
       Latitude (field 6) and Longitude (field 8) require at least 8 commas */
    if (commaFoundCount < 8) {
        if (enableDebugPrint) {
            Serial.println("Error: Insufficient fields in +CGNSSINFO response");
        }
        return googleMapUrl;
    }

    /* Extract raw latitude string from field 6 */
    String latitudeRawValue = gnsInfoData.substring(commaIndices[4] + 1, commaIndices[5]);
    
    /* Extract latitude direction (N or S) from field 7 */
    String latitudeDirection = gnsInfoData.substring(commaIndices[5] + 1, commaIndices[6]);
    
    /* Extract raw longitude string from field 8 */
    String longitudeRawValue = gnsInfoData.substring(commaIndices[6] + 1, commaIndices[7]);
    
    /* Extract longitude direction (E or W) from field 9 */
    String longitudeDirection = gnsInfoData.substring(commaIndices[7] + 1, commaIndices[8]);

    /* Convert raw latitude string to float (decimal degrees format) */
    float latitudeDecimalValue = latitudeRawValue.toFloat();
    
    /* Convert raw longitude string to float (decimal degrees format) */
    float longitudeDecimalValue = longitudeRawValue.toFloat();

    /* Split latitude decimal value into degrees, minutes, seconds */
    int latitudeDegrees = (int)latitudeDecimalValue;
    float latitudeMinutesFloat = (latitudeDecimalValue - latitudeDegrees) * 60.0F;
    int latitudeMinutes = (int)latitudeMinutesFloat;
    float latitudeSeconds = (latitudeMinutesFloat - latitudeMinutes) * 60.0F;

    /* Split longitude decimal value into degrees, minutes, seconds */
    int longitudeDegrees = (int)longitudeDecimalValue;
    float longitudeMinutesFloat = (longitudeDecimalValue - longitudeDegrees) * 60.0F;
    int longitudeMinutes = (int)longitudeMinutesFloat;
    float longitudeSeconds = (longitudeMinutesFloat - longitudeMinutes) * 60.0F;

    /* Buffers to store formatted DMS strings for latitude and longitude */
    char latitudeDmsFormatted[20];
    char longitudeDmsFormatted[20];

    /* Format latitude DMS into string including degree, minute, second and hemisphere */
    snprintf(latitudeDmsFormatted, sizeof(latitudeDmsFormatted), "%d°%d'%.1f\"%s",
             latitudeDegrees, latitudeMinutes, latitudeSeconds, latitudeDirection.c_str());

    /* Format longitude DMS into string including degree, minute, second and hemisphere */
    snprintf(longitudeDmsFormatted, sizeof(longitudeDmsFormatted), "%d°%d'%.1f\"%s",
             longitudeDegrees, longitudeMinutes, longitudeSeconds, longitudeDirection.c_str());

    /* Adjust latitude string for southern hemisphere (negative value) */
    if (latitudeDirection == "S") {
        latitudeRawValue = "-" + latitudeRawValue;
    }

    /* Adjust longitude string for western hemisphere (negative value) */
    if (longitudeDirection == "W") {
        longitudeRawValue = "-" + longitudeRawValue;
    }

    /* Generate final Google Maps URL using latitude and longitude in decimal degrees */
    googleMapUrl = "https://www.google.com/maps?q=" + latitudeRawValue + "," + longitudeRawValue;

    /* If debug printing is enabled, show DMS values and Google Maps URL */
    if (enableDebugPrint) {
        Serial.print("Coordinates (DMS): ");
        Serial.print(latitudeDmsFormatted);
        Serial.print(" ");
        Serial.println(longitudeDmsFormatted);

        Serial.print("Google Maps URL: ");
        Serial.println(googleMapUrl);
    }

    /* Return the generated Google Maps link */
    return googleMapUrl;
}

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
void requestGpsLocation(unsigned long &systemCurrentTimeMs, unsigned long intervalMs, bool debug) {
  /* Check if the interval has passed since last request */
  if ((millis() - systemCurrentTimeMs) > intervalMs) {
    /* Update the system timestamp */
    systemCurrentTimeMs = millis();

    /* Send AT command to get GNSS (GPS) information */
    String gpsResponse = sendData("AT+CGNSSINFO", 1000, debug);

    /* Parse the GPS response into a map link (custom function) */
    parseGpsToMapLink(gpsResponse, debug);
  }

  /* Forward data from debug Serial (USB) to GSM serial port */
  while (Serial.available() > 0) {
    gsmSerialPort.write(Serial.read());
    yield(); /* Allow background tasks to run */
  }

  /* Forward data from GSM serial port to debug Serial (USB) */
  while (gsmSerialPort.available() > 0) {
    Serial.write(gsmSerialPort.read());
    yield(); /* Allow background tasks to run */
  }
}
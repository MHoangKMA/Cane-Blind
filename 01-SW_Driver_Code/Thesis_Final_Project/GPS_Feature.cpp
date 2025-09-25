#include "GPS_Feature.h"

/* Global system timestamp (milliseconds) */
unsigned long systemCurrentTimeMs;

/* Variable to store the final generated Google Maps URL */
String googleMapUrl = "";

/* Externally defined HardwareSerial instance for GSM/GNSS communication */
extern HardwareSerial gsmSerialPort;

/****************************************************************************************
 *  Function Name    : sendData
 *  Description      : Sends an AT command over GSM serial port and collects the response.
 *                     The function waits until the specified timeout expires or until no more
 *                     data is available in the serial buffer. Optionally, the received response
 *                     can be printed to the debug serial port.
 *
 *  Input Parameters :
 *    atCommand       - (String) The AT command to be sent to the module.
 *    timeoutMs       - (const int) The maximum waiting time in milliseconds for the response.
 *    enableDebug     - (boolean) If true, the response will be printed to Serial monitor.
 *
 *  Return Value     :
 *    (String) The accumulated response received from the module within the timeout period.
 ****************************************************************************************/
String sendData(String atCommand, const int timeoutMs, boolean enableDebug)
{
    String responseBuffer = "";

    /* Send AT command to GSM/GNSS module */
    gsmSerialPort.println(atCommand);

    /* Record the start time for timeout calculation */
    unsigned long startTimeMs = millis();

    /* Collect response until timeout expires */
    while ((startTimeMs + timeoutMs) > millis())
    {
        while (gsmSerialPort.available() > 0)
        {
            char receivedChar = gsmSerialPort.read();
            responseBuffer += receivedChar;
        }
    }

    /* Optionally print the response for debugging */
    if (enableDebug)
    {
        Serial.print(responseBuffer);
    }

    return responseBuffer;
}

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
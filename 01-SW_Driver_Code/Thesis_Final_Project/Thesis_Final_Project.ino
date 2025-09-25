#include <Arduino.h>

/* Hardware serial instance for communication with GSM/GNSS module */
HardwareSerial gsmSerialPort(2);

/* Debug mode enable flag */
#define DEBUG_MODE_ENABLED   true

/* Pin mapping for GSM/GNSS module (UART and control pins) */
#define PIN_GSM_UART_RECEIVE           16  /* UART RX pin for GSM/GNSS module */
#define PIN_GSM_UART_TRANSMIT          17  /* UART TX pin for GSM/GNSS module */
#define PIN_GSM_POWER_KEY_CONTROL      12  /* Power key control pin for GSM/GNSS module */
#define PIN_GSM_RESET_CONTROL          14  /* Reset control pin for GSM/GNSS module */

/* Global system timestamp (milliseconds) */
unsigned long systemCurrentTimeMs;

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
 *  Function Name    : setup
 *  Description      : Arduino system initialization function. It is executed once at 
 *                     startup and performs initialization of serial ports, GSM/GNSS 
 *                     hardware control pins, and power-on sequence. The function also 
 *                     sends initial AT commands to verify communication with the GSM/GNSS 
 *                     module and starts the GNSS (GPS) subsystem for location testing.
 *
 *  Input Parameters :
 *    None
 *
 *  Return Value     :
 *    None
 ****************************************************************************************/
void setup() 
{
    /* Initialize the debug serial port at baud rate 115200 */
    Serial.begin(115200);

    /* Print initial startup message on debug serial */
    Serial.print(F("Hello! ESP32-S3 AT command V1.0 Test"));

    /* Initialize the GSM/GNSS serial port (UART2) with baud rate 115200, 
       8 data bits, no parity, 1 stop bit, and assigned RX/TX pins */
    gsmSerialPort.begin(115200, SERIAL_8N1, PIN_GSM_UART_RECEIVE, PIN_GSM_UART_TRANSMIT);

    /* Configure GSM reset pin as output */
    pinMode(PIN_GSM_RESET_CONTROL, OUTPUT);

    /* Drive GSM reset pin LOW to ensure module reset state */
    digitalWrite(PIN_GSM_RESET_CONTROL, LOW);

    /* Configure GSM power key pin as output */
    pinMode(PIN_GSM_POWER_KEY_CONTROL, OUTPUT);

    /* Drive GSM power key pin HIGH to start power sequence */
    digitalWrite(PIN_GSM_POWER_KEY_CONTROL, HIGH);

    /* Delay 3 seconds to allow the module to process power-on signal */
    delay(3000);

    /* Drive GSM power key pin LOW to complete the power-on sequence */
    digitalWrite(PIN_GSM_POWER_KEY_CONTROL, LOW);

    /* Print message indicating the start of LTE CAT1 test */
    Serial.println("ESP32-S3 4G LTE CAT1 Test Start!");

    /* Delay 2 seconds for module stabilization */
    delay(2000);

    /* Delay an additional 2 seconds for module stabilization */
    delay(2000);

    /* Print message indicating waiting for network registration */
    Serial.println("Wait a few minutes for 4G start");

    /* Delay 3 seconds before sending AT commands */
    delay(3000);

    /* Send AT command to check communication with module */
    sendData("AT", 1000, DEBUG_MODE_ENABLED);

    /* Send AT command to get ICCID of the SIM card */
    sendData("AT+CICCID", 1000, DEBUG_MODE_ENABLED);

    /* Send AT command to get module information */
    sendData("AT+SIMCOMATI", 1000, DEBUG_MODE_ENABLED);

    /* Send AT command to check operator information */
    sendData("AT+COPS?", 1000, DEBUG_MODE_ENABLED); 

    /* Send AT command to get firmware version */
    sendData("AT+GMR", 1000, DEBUG_MODE_ENABLED); 

    #if 1
    /* Send AT command to enable GNSS power */
    sendData("AT+CGNSSPWR=1", 1000, DEBUG_MODE_ENABLED);

    /* Delay 12 seconds to wait until "+CGNSSPWR: READY!" is reported */
    delay(12000);

    /* Send AT command to set GNSS baud rate to 9600 */
    sendData("AT+CGNSSIPR=9600", 1000, DEBUG_MODE_ENABLED);

    /* Send AT command to start GNSS test stream */
    sendData("AT+CGNSSTST=1", 1000, DEBUG_MODE_ENABLED);

    /* Send AT command to get GNSS information */
    sendData("AT+CGNSSINFO", 1000, DEBUG_MODE_ENABLED); 
    #endif

    /* Store current system uptime in milliseconds */
    systemCurrentTimeMs = millis();
}

/****************************************************************************************
 *  Function Name    : loop
 *  Description      : Arduino main execution loop. This function runs continuously after 
 *                     system initialization. It periodically requests GNSS (GPS) information 
 *                     from the GSM/GNSS module every 30 seconds and generates a Google Maps 
 *                     URL from the response. In addition, it forwards data between the debug 
 *                     Serial (USB) and the GSM serial port, enabling transparent communication 
 *                     for debugging and testing.
 *
 *  Input Parameters :
 *    None
 *
 *  Return Value     :
 *    None
 ****************************************************************************************/
void loop() 
{
    /* Check if 30 seconds have passed since the last GNSS information request */
    if ((millis() - systemCurrentTimeMs) > 30000UL)
    {
        /* Update the system timestamp for the next interval */
        systemCurrentTimeMs = millis();

        /* Send AT command to get GNSS (GPS) information and parse to map link */
        String gpsResponse = sendData("AT+CGNSSINFO", 1000, DEBUG_MODE_ENABLED);
        parseGpsToMapLink(gpsResponse, DEBUG_MODE_ENABLED);
    }

    /* Forward data from debug Serial (USB) to GSM serial port */
    while (Serial.available() > 0) 
    {
        gsmSerialPort.write(Serial.read());
        yield(); /* Allow background tasks to run */
    }

    /* Forward data from GSM serial port to debug Serial (USB) */
    while (gsmSerialPort.available() > 0) 
    {
        Serial.write(gsmSerialPort.read());
        yield(); /* Allow background tasks to run */
    }
}
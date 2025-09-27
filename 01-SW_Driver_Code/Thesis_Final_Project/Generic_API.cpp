#include "Generic_API.h"


/* Global system timestamp (milliseconds) */
unsigned long systemCurrentTimeMs =0U;


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

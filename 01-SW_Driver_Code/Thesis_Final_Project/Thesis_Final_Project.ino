#include <Arduino.h>

/* Hardware serial instance for communication with GSM/GNSS module */
HardwareSerial gsmSerialPort(2);

/* Debug mode enable flag */
#define DEBUG_MODE_ENABLED   true

/* Pin mapping for GSM/GNSS module (UART and control pins) */
#define PIN_GSM_UART_RX      16
#define PIN_GSM_UART_TX      17
#define PIN_GSM_PWRKEY       12
#define PIN_GSM_RESET        14

/* Global system timestamp (milliseconds) */
unsigned long systemCurrentTimeMs;


/****************************************************************************************
 *  Function Name    : sendData
 *  Description      : Sends an AT command over Serial2 (mySerial2) and collects the response.
 *                     The function waits until the specified timeout expires or until no more
 *                     data is available in the serial buffer. Optionally, the received response
 *                     can be printed to the debug serial port.
 *
 *  Input Parameters :
 *    atCommand   - (String) The AT command to be sent to the module.
 *    timeoutMs   - (const int) The maximum waiting time in milliseconds for the response.
 *    enableDebug - (boolean) If true, the response will be printed to Serial monitor.
 *
 *  Return Value   :
 *    (String) The accumulated response received from the module within the timeout period.
 ****************************************************************************************/

String sendData(String atCommand, const int timeoutMs, boolean enableDebug)
{
  String responseBuffer = "";
  mySerial2.println(atCommand);

  unsigned long startTimeMs = millis();
  while ((startTimeMs + timeoutMs) > millis())
  {
    while (mySerial2.available() > 0)
    {
      char receivedChar = mySerial2.read();
      responseBuffer += receivedChar;
    }
  }

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
 *  Return Value   :
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
    gsmSerialPort.begin(115200, SERIAL_8N1, PIN_GSM_UART_RX, PIN_GSM_UART_TX);

    /* Configure GSM reset pin as output */
    pinMode(PIN_GSM_RESET, OUTPUT);

    /* Drive GSM reset pin LOW to ensure module reset state */
    digitalWrite(PIN_GSM_RESET, LOW);

    /* Configure GSM power key pin as output */
    pinMode(PIN_GSM_PWRKEY, OUTPUT);

    /* Drive GSM power key pin HIGH to start power sequence */
    digitalWrite(PIN_GSM_PWRKEY, HIGH);

    /* Delay 3 seconds to allow the module to process power-on signal */
    delay(3000);

    /* Drive GSM power key pin LOW to complete the power-on sequence */
    digitalWrite(PIN_GSM_PWRKEY, LOW);

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


void loop() 
{
    /* Check if 30 seconds have passed since the last GNSS information request */
    if ((millis() - systemCurrentTimeMs) > 30000UL)
    {
        /* Update the system timestamp for the next interval */
        systemCurrentTimeMs = millis();

        /* Send AT command to get GNSS (GPS) information */
        sendData("AT+CGNSSINFO", 1000, DEBUG_MODE_ENABLED);
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


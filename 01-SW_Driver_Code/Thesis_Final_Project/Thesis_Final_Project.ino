#include "GPS_Feature.h"
#include "CALL_Feature.h"

/* Hardware serial instance for communication with GSM/GNSS module */
HardwareSerial gsmSerialPort(2);

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

    pinMode(BUTTON_PIN, INPUT_PULLUP);

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

    #if 1
  sendData("AT", 2000, DEBUG_MODE_ENABLED);
  sendData("AT+CPIN?", 2000, DEBUG_MODE_ENABLED);   // kiểm tra SIM sẵn sàng
  sendData("AT+CSQ", 2000, DEBUG_MODE_ENABLED);     // kiểm tra tín hiệu
  sendData("AT+COPS?", 2000, DEBUG_MODE_ENABLED);   // kiểm tra đã đăng ký mạng chưa
  sendData("AT+CLIP=1", 1000, DEBUG_MODE_ENABLED);  // bật hiển thị số gọi đến
#endif
 
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
  /* Call the GPS request handler function */
   requestGpsLocation(systemCurrentTimeMs, 30000UL, DEBUG_MODE_ENABLED);

   handleCallAndATPassthrough(BUTTON_PIN, DEBUG_MODE_ENABLED);
}
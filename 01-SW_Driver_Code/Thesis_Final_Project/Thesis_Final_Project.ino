#include <Arduino.h>
#include "GPS_Feature.h"

/* Hardware serial instance for communication with GSM/GNSS module */
HardwareSerial gsmSerialPort(2);

/* Pin mapping for GSM/GNSS module (UART and control pins) */
#define PIN_GSM_UART_RECEIVE           16  /* UART RX pin for GSM/GNSS module */
#define PIN_GSM_UART_TRANSMIT          17  /* UART TX pin for GSM/GNSS module */
#define PIN_GSM_POWER_KEY_CONTROL      12  /* Power key control pin for GSM/GNSS module */
#define PIN_GSM_RESET_CONTROL          14  /* Reset control pin for GSM/GNSS module */

#define BUTTON_PIN 35

#define DEBOUNCE_DELAY 50    /* Debounce time in milliseconds */

/* Variables for button press tracking */
unsigned long pressStart = 0;     /* Timestamp when button was pressed down */
bool isPressed = false;           /* Flag indicating if button is currently held */

/* Variables for software debouncing */
int lastRawState = HIGH;          /* Last raw (unfiltered) state read from the pin */
int stableState = HIGH;           /* Stable (debounced) state of the button */
unsigned long lastDebounceTime = 0; /* Last time the raw state was toggled */


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

/****************************************************************************************
 *  Function Name    : debounceRead
 *  Description      : Reads the state of a button and applies software debouncing 
 *                     to eliminate false triggering caused by mechanical bouncing.
 *                     
 *                     - The function monitors changes in the raw button signal.
 *                     - If a change is detected, it resets a timer.
 *                     - Only when the signal remains stable longer than DEBOUNCE_DELAY,
 *                       the "stable state" is updated and returned.
 *
 *  Input            : pin (int) - The GPIO pin number where the button is connected.
 *
 *  Output           : int (LOW or HIGH) - The stable and debounced state of the button.
 *
 *  Note             : 
 *    - Use INPUT_PULLUP mode for the button pin to ensure stable logic.
 *    - This function must be called repeatedly in the main loop to work correctly.
 ****************************************************************************************/
int debounceRead(int pin) {
  int currentRawState = digitalRead(pin);    /* Read the raw (unfiltered) state from the pin */

  /* If the raw state changes compared to the last raw read, 
     reset the debounce timer to start measuring stability time */
  if (currentRawState != lastRawState) {
    lastDebounceTime = millis();
  }

  /* If the signal has remained unchanged for longer than DEBOUNCE_DELAY,
     update the stable state to reflect the new value */
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (currentRawState != stableState) {
      stableState = currentRawState;
    }
  }

  /* Store the raw state for comparison in the next function call */
  lastRawState = currentRawState;

  /* Return the stable, debounced state of the button */
  return stableState;
}

/****************************************************************************************
 *  Function Name    : handleCallAndATPassthrough
 *  Description      : Manages button press events with software debouncing to trigger 
 *                     phone calls and performs AT command passthrough between the 
 *                     Serial Monitor (USB) and the GSM/GNSS module.
 *  
 *                     - Uses debounceRead() to obtain a stable button state.
 *                     - Detects short press durations to print different states.
 *                     - Detects a long press and initiates a phone call sequence:
 *                          1. Dial a predefined phone number.
 *                          2. Keep the call active for 20 seconds.
 *                          3. Hang up the call automatically.
 *                     - Independently of the button logic, continuously forwards AT 
 *                       commands from the Serial Monitor to the GSM/GNSS module and 
 *                       relays the module responses back to the Serial Monitor.
 *
 *  Input Parameters :
 *    - buttonInputPin        : int 
 *                              GPIO pin number connected to the button input.
 *
 *    - enableDebugMessages   : bool 
 *                              When true, enables printing of debug messages to 
 *                              the Serial Monitor.
 *
 *  Return Value     : None
 *
 ****************************************************************************************/
void handleCallAndATPassthrough(int buttonInputPin, bool enableDebugMessages) 
{
    int stableState = debounceRead(buttonInputPin);  

    /* Detect button press (falling edge) */
    if (stableState == LOW && !isPressed) {
        pressStart = millis();
        isPressed = true;
    }
    /* Detect button release (rising edge) */
    else if (stableState == HIGH && isPressed) {
        unsigned long duration = millis() - pressStart;

        /* Handle short press states */
        if (duration < 300) {
            Serial.println("State 1 detected");
        } 
        else if (duration <= 800) {
            Serial.println("State 2 detected");
        } 
        else {
            /* ------------------- Call Sequence ------------------- */
            Serial.println("Dialing number...");

            /* Send AT command to dial number (replace with your phone number) */
            sendData("ATD0387695355;", 5000, enableDebugMessages);

            /* Keep call active for 20 seconds (blocking delay) */
            delay(20000);

            /* Send AT command to hang up */
            sendData("AT+CHUP", 2000, enableDebugMessages);

            Serial.println("Call ended.");
        }
        isPressed = false;
    }

    /* ------------------- AT Passthrough Logic ------------------- */

    /* Forward AT commands from Serial Monitor (USB) to GSM/GNSS module */
    while (Serial.available() > 0) {
        gsmSerialPort.write(Serial.read());  
        yield();   /* Allow background tasks to run */
    }

    /* Forward responses from GSM/GNSS module back to Serial Monitor (USB) */
    while (gsmSerialPort.available() > 0) {
        Serial.write(gsmSerialPort.read());  
        yield();   /* Allow background tasks to run */
    }
}


void handleCallAndATPassthrough2(int buttonInputPin, bool enableDebugMessages) 
{
 
     if (digitalRead(BUTTON_PIN) == LOW) {
    Serial.println("Dialing number...");
    sendData("ATD0387695355;", 5000, enableDebugMessages);  // thay số bạn muốn gọi
    delay(20000);                             // giữ cuộc gọi 20s
    sendData("AT+CHUP", 2000, enableDebugMessages);         // cúp máy
    Serial.println("Call ended.");
  }
    /* -------------------- Serial Passthrough Logic -------------------- */

    /* Forward AT commands from Serial Monitor (USB) to GSM/GNSS module */
    while (Serial.available() > 0) {
        gsmSerialPort.write(Serial.read());  /* Send one byte at a time */
        yield();                                 /* Allow background tasks to run */
    }

    /* Forward responses from GSM/GNSS module back to Serial Monitor (USB) */
    while (gsmSerialPort.available() > 0) {
        Serial.write(gsmSerialPort.read());  /* Print module response byte-by-byte */
        yield();                                 /* Allow background tasks to run */
    }
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
  //  requestGpsLocation(systemCurrentTimeMs, 30000UL, DEBUG_MODE_ENABLED);

   handleCallAndATPassthrough(BUTTON_PIN, DEBUG_MODE_ENABLED);
}
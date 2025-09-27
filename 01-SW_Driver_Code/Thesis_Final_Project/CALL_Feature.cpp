#include "CALL_Feature.h"


/* Variables for button press tracking */
unsigned long pressStart = 0;     /* Timestamp when button was pressed down */
bool isPressed = false;           /* Flag indicating if button is currently held */

/* Variables for software debouncing */
int lastRawState = HIGH;          /* Last raw (unfiltered) state read from the pin */
int stableState = HIGH;           /* Stable (debounced) state of the button */
unsigned long lastDebounceTime = 0; /* Last time the raw state was toggled */

/****************************************************************************************
 *  Function Name    : debounceRead
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
        else if (duration <= 650) {
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

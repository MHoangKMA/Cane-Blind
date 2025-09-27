#ifndef CALL_FEATURE_H
#define GPS_FEATURE_H

#include "Generic_API.h"

/* ================== Configuration ================== */
#define DEBOUNCE_DELAY 50    /* Debounce time in milliseconds */
#define BUTTON_PIN 35

/* ================== Global Variables ================== */
extern unsigned long pressStart;      /* Timestamp when button was pressed down */
extern bool isPressed;                /* Flag indicating if button is currently held */
extern int lastRawState;              /* Last raw state read from the button pin */
extern int stableState;               /* Stable debounced state of the button */
extern unsigned long lastDebounceTime;/* Last debounce timestamp */

/* ================== Function Prototypes ================== */

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
int debounceRead(int pin);

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
void handleCallAndATPassthrough(int buttonInputPin, bool enableDebugMessages);

#endif /* CALL_FEATURE_H */

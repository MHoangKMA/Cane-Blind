#ifndef GENERIC_API_H
#define GENERIC_API_H

#include <Arduino.h>

/* Pin mapping for GSM/GNSS module (UART and control pins) */
#define PIN_GSM_UART_RECEIVE           16  /* UART RX pin for GSM/GNSS module */
#define PIN_GSM_UART_TRANSMIT          17  /* UART TX pin for GSM/GNSS module */
#define PIN_GSM_POWER_KEY_CONTROL      12  /* Power key control pin for GSM/GNSS module */
#define PIN_GSM_RESET_CONTROL          14  /* Reset control pin for GSM/GNSS module */

/* Debug mode enable flag */
#define DEBUG_MODE_ENABLED   true

/* Externally defined HardwareSerial instance for GSM/GNSS communication */
extern HardwareSerial gsmSerialPort;

/* Function declarations */
String sendData(String atCommand, const int timeoutMs, boolean enableDebug);

#endif
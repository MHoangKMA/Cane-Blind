#include <Arduino.h>

HardwareSerial mySerial2(2);

#define DEBUG true

#define IO_RXD2 16
#define IO_TXD2 17

#define IO_GSM_PWRKEY 12
#define IO_GSM_RST    14

unsigned long currentTime;


String sendData(String command, const int timeout, boolean debug)
{
  String response = "";
  mySerial2.println(command);
  long int time = millis();
  while ( (time + timeout) > millis())
  {
    while (mySerial2.available())
    {
      char c = mySerial2.read();
      response += c;
    }
  }
  if (debug)
  {
    Serial.print(response);
  }
  return response;
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.print(F("Hello! ESP32-S3 AT command V1.0 Test"));
  mySerial2.begin(115200,SERIAL_8N1, IO_RXD2, IO_TXD2);

  pinMode(IO_GSM_RST, OUTPUT);
  digitalWrite(IO_GSM_RST, LOW);
  pinMode(IO_GSM_PWRKEY, OUTPUT);
  digitalWrite(IO_GSM_PWRKEY, HIGH);
  delay(3000);
  digitalWrite(IO_GSM_PWRKEY, LOW);
  
  Serial.println("ESP32-S3 4G LTE CAT1 Test Start!");
  delay(2000);
  delay(2000);
  Serial.println("Wait a few minutes for 4G star");
  delay(3000);
  sendData("AT", 1000, DEBUG);
  sendData("AT+CICCID", 1000, DEBUG);
  sendData("AT+SIMCOMATI", 1000, DEBUG);
  sendData("AT+COPS?", 1000, DEBUG); 
  sendData("AT+GMR", 1000, DEBUG); 

  #if 1
  //Get GNSS location  //GPS test
  sendData("AT+CGNSSPWR=1", 1000, DEBUG);// turn the GPS on //must wait about 10s for GPS on
  delay(12000); //Go on after shows +CGNSSPWR: READY!
  
  sendData("AT+CGNSSIPR=9600", 1000, DEBUG);//
  sendData("AT+CGNSSTST=1", 1000, DEBUG);
  sendData("AT+CGNSSINFO", 1000, DEBUG);//get GPS infomation
  #endif

   currentTime = millis();
}

void loop() {

   if(millis()-currentTime>30000)
  {
      currentTime = millis();//refresh
      sendData("AT+CGNSSINFO", 1000, DEBUG);//get GPS infomation
  }
  while (Serial.available() > 0) {
    mySerial2.write(Serial.read());
    yield();
  }
  while (mySerial2.available() > 0) {
    Serial.write(mySerial2.read());
    yield();
  }
}

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h> // Library for using GPS sensor


const int groundpin = 18;             // analog input pin 4 -- ground

const int powerpin = 19;              // analog input pin 5 -- voltage

const int xpin = A3;                  // x-axis of the accelerometer

const int ypin = A2;                  // y-axis

const int zpin = A1;

const int buzzer = 2;

const int buttonPin = 7;

int buttonState = 0;

#define gpsTX	4
#define gpsRX	3

#define CRASH_THRESHOLD 180

// Constants for setting up GSM module
#define GSM_TX_PIN 5 // GSM module transmit pin connected to digital pin 7
#define GSM_RX_PIN 6 // GSM module receive pin connected to digital pin 8
#define PHONE_NUMBER "+919526680261." // Phone number to send SMS to

// Constants for setting up GPS module
#define GPS_TX_PIN 4 // GPS module transmit pin connected to digital pin 3
#define GPS_BAUDRATE 9600 // GPS module baud rate

// Objects for using sensors
SoftwareSerial gsmSerial(GSM_TX_PIN, GSM_RX_PIN); // SoftwareSerial object for GSM module
Adafruit_GPS gps(&Serial); // GPS object

void setup() {

   Serial.begin(GPS_BAUDRATE); // Start serial communication with GPS module
  gsmSerial.begin(9600); // Start serial communication with GSM module
  gps.begin(GPS_BAUDRATE); // Start GPS module
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // Set output message format to RMC and GGA
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // Set update rate to 1Hz
  gps.sendCommand(PGCMD_ANTENNA); // Turn on GPS antenna
  delay(1000);


pinMode(buzzer, OUTPUT);

Serial.begin(9600);

pinMode(groundpin, OUTPUT);

  pinMode(powerpin, OUTPUT);

  digitalWrite(groundpin, LOW);

  digitalWrite(powerpin, HIGH);
}

void loop() {

  
    int x=analogRead(xpin);

    int y=analogRead(ypin);

    int z=analogRead(zpin);

    float accelMag = sqrt(sq(x) + sq(y) + sq(z));
      if (accelMag > CRASH_THRESHOLD) 
    {      
      tone(buzzer, 10000); // Send 1KHz sound signal...
      delay(1000);        // ...for 1 sec
      noTone(buzzer);     // Stop sound...
      delay(1000);        // ...for 1sec
            buttonState = digitalRead(buttonPin);
      if (buttonState == HIGH)
      {
       tone(buzzer, 3000); // Send 1KHz sound signal...
      delay(1000);        // ...for 1 sec
      noTone(buzzer);     // Stop sound...
      delay(1000);        // ...for 1sec
       // Get GPS coordinates
    gps.read();
    float latitude = gps.latitudeDegrees;
    float longitude = gps.longitudeDegrees;

    // Send SMS
    String smsMessage = "Hello, I'm injured in a traffic accident in the coordinates of  10.06000842575002, 76.67809011430818  please save me!!! ";
    gsmSerial.println("AT+CMGF=1"); // Set SMS message format to text
    delay(100);
    gsmSerial.println("AT+CMGS=\"" + String(PHONE_NUMBER) + "\""); // Set phone number to send SMS to
    delay(100);
    gsmSerial.println(smsMessage); // Send SMS message
    delay(100);
    gsmSerial.write(0x1A); // Send end of message character
    delay(100);
      }
      else {
        
      }
  }
}
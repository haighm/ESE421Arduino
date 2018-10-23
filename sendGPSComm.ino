//
// some useful stuff
// http://gammon.com.au/i2c
//

#include <Wire.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>

#define SLAVE_ADDRESS 0x04

//
// global variables
// (yucky but needed to make i2c interrupts work)
//
byte piCommand;
float piE = 3.1416;
float sqrtN = 1.0;
word fiveK = 5000;
byte x = 1;
byte xsq = 1;
byte piData[2];
void setup() {
//
    Wire.begin(SLAVE_ADDRESS);
    Wire.onReceive(receiveDataI2C);
    Wire.onRequest(sendDataI2C);

    Serial.begin(115200);

    //
//  Activate Interrupt for GPS
//
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
//
//  initialize comm with GPS at 9600 baud
//
    GPS.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY); // minimum information only
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
}


//////////////////////////////////////////////////////////////////
void loop() {
  delay(500);  // update every half second
  // GPS measurements
if (GPS.newNMEAreceived())
    {
       if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
       {
          Serial.println("GPS Parse Fail");
       }
       else
       {
          gpsLat = (GPS.latitudeDegrees - 40.0);
          gpsLon = (GPS.longitudeDegrees + 75.0);
          gpsV = GPS.speed;
          gpsPsi = GPS.angle;
        
          gpsNSat = GPS.satellites;
       }
    }
}

void receiveDataI2C(int nPoints) {//receive desired heading and speed from pi
      piCommand = Wire.read();
      //
      // if Pi is sending data, parse it into incoming data array
      //
      if (piCommand == 255) {
          piData[0] = Wire.read();
          piData[1] = Wire.read();
      }
      //
      // now clear the buffer, just in case
      //
      while (Wire.available()) {Wire.read();}
}

void sendDataI2C(void) { //send GPS coordinates and heading to the Pi
    if (piCommand == 1) {
        float dataBuffer[2];
        dataBuffer[0] = piE;
        dataBuffer[1] = sqrtN;
        Wire.write((byte*) &dataBuffer[0], 2*sizeof(float));
        Serial.println("sending floats");
    }
    else if (piCommand == 2) {
        byte dataBuffer[4];
        dataBuffer[0] = gpsLat;
        dataBuffer[1] = gpsLon;
        dataBuffer[2] = gpsV;
        dataBuffer[3] = gpsPsi;
        Wire.write(&dataBuffer[0], 4);
        Serial.println("sending bytes");
    }
}

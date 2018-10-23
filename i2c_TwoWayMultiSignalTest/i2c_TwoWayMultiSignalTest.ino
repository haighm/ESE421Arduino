//
// some useful stuff
// http://gammon.com.au/i2c
//

#include <Wire.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
float psiDesired = 131; //angle of lines on tennis court
byte motorPWM=255;
#define pingTrigPin 23 // ping sensor trigger pin (output from Arduino)
#define pingEchoPin 25 // ping sensor echo pin (input to Arduino)
#define pingGrndPin 27 // ping sensor ground pin (use digital pin as ground)
#define motorPin 8 // PWM for motor
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>

//
// IMU uses SPI -- here are the pins on the Mega
// (Pins 49 & 47 are user selection)
//
#define LSM9DS1_SCK 52  //BDK-mega
#define LSM9DS1_MISO 50 //BDK-mega
#define LSM9DS1_MOSI 51 //BDK-mega
#define LSM9DS1_XGCS 49 //BDK-mega
#define LSM9DS1_MCS 47 //BDK-mega
#define servoPin 7 // pin for servo signal
#include <Servo.h>

//
// global variables
// (yucky but needed to make i2c interrupts work later)
//
Servo steeringServo;
byte delta_bias = 84; //servo is not exactly at 90 degrees
float pingDistanceCM = 0.0;
float gyroAngle = 84; //servo is not exactly at 90 degrees
float psiGyro = 0;
float estHeadingAngle = 0.0;
float gyroBias = 0.0;
float k = 0.25;
float servoAngleDeg = 0.0;
float psiX = 0;
float vel = 0;
float psiEstimated = 0.0;
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);

#define SLAVE_ADDRESS 0x04

//
// global variables
// (yucky but needed to make i2c interrupts work)
float gpsLat;
float gpsLon;
float gpsV;
float gpsPsi;
int gpsNSat;

byte piCommand = 1;
float piE = 3.1416;
float sqrtN = 1.0;
word fiveK = 5000;
byte x = 1;
byte xsq = 1;
byte piData[2];

//
// connect GPS to Hardware Serial1 (Serial0 is for USB)
// Serial1 is pins 18 & 19 on Mega
//
HardwareSerial mySerial = Serial1;
Adafruit_GPS GPS(&Serial1);
void setup() {
  Serial.begin(115200);
  
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
//  set up the ping sensor pins
//
    pinMode(pingGrndPin,OUTPUT); digitalWrite(pingGrndPin,LOW);
    pinMode(pingTrigPin,OUTPUT);
    pinMode(pingEchoPin,INPUT);
    pinMode(motorPin,OUTPUT);

// initialize gyro / mag / accel
//
    if (!lsm.begin())
    {
      Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
      while (1);
    }
    Serial.println("Found LSM9DS1 9DOF");
    //
    // set ranges for sensor
    //
    lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
    lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
    lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
    steeringServo.attach(servoPin);
    float r = 0.0;
    int t = 0;

    while (t < 40)
     { 
      lsm.read();  /* ask it to read in the data */
      sensors_event_t a, m, g, temp;
      lsm.getEvent(&a, &m, &g, &temp);
      r += g.gyro.z;
      delay(50);
      t += 1;
      } 
      gyroBias = r/40;
}

SIGNAL(TIMER0_COMPA_vect) {
   char c = GPS.read();
}

void loop() {

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
////////////////////////////////////////////////////////////
    //  get the ping distance
    getPingDistanceCM();
    lsm.read();  /* ask it to read in the data */
    sensors_event_t a, m, g, temp;
    lsm.getEvent(&a, &m, &g, &temp);

    //emergency stop 
    if(pingDistanceCM < 30){
          analogWrite(motorPin,0);
    }
    else {
    analogWrite(motorPin, 150);}
////////////////////////////////////////////////////////////
// implement control laws 

    float deltaT = 0.001*50; //deltaT = 50 ms
    float delta = 0; 
    float tau = 5;
      
    psiGyro += (g.gyro.z - gyroBias) * deltaT; 
    psiX += (deltaT/tau)*(gpsPsi-psiGyro-psiX); //implementation of complementary filter through low pass filter
    psiEstimated = psiGyro+psiX; // final output complementary filter
    
    float unwrapped = psiDesired-psiEstimated; // fixes wrapping of angle
    if (unwrapped > 180) {  //determines which direction requires less turning
      unwrapped -= 360;
    }
    
    delta = k*(unwrapped) + delta_bias; //desired steering angle 
    steeringServo.write(constrain(delta,60,120));
//  pause 0.050 seconds
    delay(50);
}

void receiveDataI2C(int nPoints) {
    
      //piCommand = Wire.read();
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

void sendDataI2C(void) {
    if (piCommand == 1) {
        float dataBuffer[2];
        dataBuffer[0] = (byte)psiEstimated;
        dataBuffer[1] = (byte)vel;
        Wire.write((byte*) &dataBuffer[0], 2*sizeof(float));
    }
    else if (piCommand == 2) {
        byte dataBuffer[4];
        dataBuffer[0] = 100;
        dataBuffer[1] = 100;
        dataBuffer[2] = x;
        dataBuffer[3] = xsq;
        Wire.write(&dataBuffer[0], 4);
    }
}

////////////////////////////////////////////////////////////
// Ping Sensor -- update value of pingDistanceCM
////////////////////////////////////////////////////////////
void getPingDistanceCM()
{
  //
  // 3000 us timeout implies maximum distance is 51cm
  // but in practice, actual max larger?
  //
  const long timeout_us = 3000;
  //
  // pingTrigPin = trigger pin
  // pingEchoPin = echo pin
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  //
  digitalWrite(pingTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingTrigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingTrigPin, LOW);
  //
  // The echo pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  //
  unsigned long echo_time;
  echo_time = pulseIn(pingEchoPin, HIGH, timeout_us);
  if (echo_time == 0)
 {
    echo_time = timeout_us;
 }
  //
  // return the distance in centimeters
  // distance = (10^-6) * (echo_time_us) * (speed of sound m/s) * (100 cm/m) / 2
  // divide by 2 because we measure "round trip" time for echo
  // (0.000001 * echo_time_us * 340.0 * 100.0 / 2.0)
  // = 0.017*echo_time
  //
  pingDistanceCM = constrain(0.017*echo_time,5.0,50.0);
}

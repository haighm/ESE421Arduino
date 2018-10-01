
#include <Servo.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_GPS.h>

#define SLAVE_ADDRESS 0x04 //Address for the pi to talk to arduino DO NOT CHANGE

#define servoPin 7 // pin for servo signalj

#define motorPin 8 // PWM for motor

//ping sensor define
#define pingTrigPin 23 // ping sensor trigger pin (output from Arduino)
#define pingEchoPin 25 // ping sensor echo pin (input to Arduino)
#define pingGrndPin 27 // ping sensor ground pin (use digital pin as ground)

//IMU sensor define
#define LSM9DS1_SCK 52  //BDK-mega
#define LSM9DS1_MISO 50 //BDK-mega
#define LSM9DS1_MOSI 51 //BDK-mega
#define LSM9DS1_XGCS 49 //BDK-mega
#define LSM9DS1_MCS 47 //BDK-mega

//
// tell sensor library which pins for accel & gyro data
//
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);



//
// global variables
// (yucky but needed to make i2c interrupts work later)
//
float SERVOANGLENEUT = 98; //set this to the neutral point of the servo
float pingDistanceCM = 0.0; //ping sensor var
float gyroZbias = 0.8; //degrees per second

//gps vars. Default all to 0
float DEF = 180;
float gpsLat = DEF;
float gpsLon = DEF;
float gpsV = DEF;
float gpsPsi = DEF;
int gpsNSat = DEF;
//
// connect GPS to Hardware Serial1 (Serial0 is for USB)
// Serial1 is pins 18 & 19 on Mega
//
HardwareSerial mySerial = Serial1;
Adafruit_GPS GPS(&Serial1);


//
// Global variables deciding what our robot should be doing
byte motorPWM=150; //needs to be between 0-255. this is motor speed
float desHeading = 0;



Servo steeringServo; //servo name variable


void setup() {
    Serial.begin(115200);       // for sending info to the terminal
//
//  make the motors not spin
//
    pinMode(motorPin,OUTPUT);
    analogWrite(motorPin,0);

    //servo setup
    steeringServo.attach(servoPin); //initalize servo to pin
    steeringServo.write(SERVOANGLENEUT);//set servo to straight line

    //gyro setup
    setupGyro();
    
//
//  set up the ping sensor pins
//
    pinMode(pingGrndPin,OUTPUT); digitalWrite(pingGrndPin,LOW);
    pinMode(pingTrigPin,OUTPUT);
    pinMode(pingEchoPin,INPUT);

    //Prepare Raspberry Pi Communication
    Wire.begin(SLAVE_ADDRESS);
    Wire.onReceive(receiveData);
    Wire.onRequest(sendData);

    //set up GPS
    setupGPS();

    Serial.begin(115200);
    
}

void loop() {
  unsigned long time = millis(); //keep track of how long its been since the loop started. want to loop every 20 milliseconds

  static float estHeading = desHeading;
  
  setSpeed(); //set the wheel speed
  updateGPSData(); //updates our GPS data for
  getDesiredHeading(); //read desired heading from the Pi

  estHeading = correctHeading(desHeading, estHeading); //keep track of the estimated heading angle and correct it as needed.

//
//  pause waiting for 20 milliseconds
//
   Serial.println();
   int DELAY = 20; //amount we wish to delay in miliseconds
   while(millis() <= time + DELAY){
      
   }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// CONTROL HELPERS
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



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


////////////////////////////////////////////////////////////
// Update Value of speed
////////////////////////////////////////////////////////////
void setSpeed() {
  //
  //Return the the value of the speed that we wish to drive at
  //
  getPingDistanceCM(); //get distance to furthest object
    
  // set wheel speed
  if(pingDistanceCM >= 30){
    motorPWM = 180; //set motor speed
  }
  else {
    motorPWM = 0; //emergency stop
  }
  analogWrite(motorPin, motorPWM);
}


////////////////////////////////////////////////////////////
// Adjust wheels to set self to correct heading
////////////////////////////////////////////////////////////
float correctHeading(float desHeading, float estHeading) {
  lsm.read();  /* ask it to read in the data */
  sensors_event_t a, m, g, temp; //a = linear acceleration , m = magnetometer reading, g = gyro acceleration
  lsm.getEvent(&a, &m, &g, &temp);
  float gyroZbias = 0.8; //degrees per second

  
  static float PsiX = gpsPsi;
  static float PsiR = gpsPsi;
  float tau = 5000;
  int deltaTms = 20; //dt
  float headingK = 1.5; //constant in controller
  

//  estHeading -=(g.gyro.z-gyroZbias)*0.001*deltaTms; //integrate
//  float servoAngleDeg = SERVOANGLENEUT + headingK *(desHeading - estHeading);
//  steeringServo.write(constrain(servoAngleDeg, SERVOANGLENEUT-25, SERVOANGLENEUT+25));

  PsiR += (g.gyro.z-gyroZbias)*deltaTms*0.001; //intertial rate integrator
  PsiX = deltaTms*0.001/tau*(gpsPsi-PsiR+ PsiX);
  
  Serial.println(estHeading);
  Serial.println(PsiR);
  Serial.println(PsiX);
  
  estHeading = (PsiR+PsiX);
  
  
  float servoAngleDeg = SERVOANGLENEUT - headingK *(desHeading - estHeading);
  Serial.println(servoAngleDeg);
  steeringServo.write(constrain(servoAngleDeg, SERVOANGLENEUT-25, SERVOANGLENEUT+25)); 
  return estHeading;
}

////////////////////////////////////////////////////////////
// Wrap angles around so that 0 = 360
////////////////////////////////////////////////////////////
float wrapAngle(float angle){
  while (angle < 0) { //while the sum of the degrees is less than 0 keep adding 360 so that we land between 0 and 360
    angle += 360;
  }
  while (angle >= 360) { //subtract too big angles to land in 0 to 360
    angle -= 360;
  }

  return angle;
}

////////////////////////////////////////////////////////////
// Read data from I2C communication from Raspberry Pi
////////////////////////////////////////////////////////////
void receiveData(int byteCount) {
  while(Wire.available()){
    
    int val = Wire.read();
    Serial.print(val);
    //steeringServo.write(constrain(val, servoAngleNeut-25, servoAngleNeut+25));
    
  }
}

////////////////////////////////////////////////////////////
// Send Data back to Raspberry Pi
////////////////////////////////////////////////////////////
void sendData() {
  
}


////////////////////////////////////////////////////////////
// Updates global variables of GPS signal
////////////////////////////////////////////////////////////
void updateGPSData(){
//   if (GPS.newNMEAreceived())
//    {
//       if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
//       {
//          Serial.println("GPS Parse Fail");
//       }
//       else
//       {
//          gpsLat = (GPS.latitudeDegrees - 40.0);
//          gpsLon = (GPS.longitudeDegrees + 75.0);
//          gpsV = GPS.speed;
//          gpsPsi = GPS.angle;
//          gpsNSat = GPS.satellites;
//       }
//    }
}


void getDesiredHeading(){
  desHeading = DEF;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP HELP
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////
// Perform Gyro Setup
////////////////////////////////////////////////////////////
void setupGyro() {
  //
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

    //get a gzbias
    float gyroAvg = 0;
    for(int i = 0; i < 100; i++) {
      lsm.read();  /* ask it to read in the data */
      sensors_event_t a, m, g, temp; //a = linear acceleration , m = magnetometer reading, g = gyro acceleration
      lsm.getEvent(&a, &m, &g, &temp);
      gyroAvg += g.gyro.z;
      delay(20);
    }
    gyroAvg /= 100;
    gyroZbias = gyroAvg;
}


////////////////////////////////////////////////////////////
// Performs GPS Setup
////////////////////////////////////////////////////////////
void setupGPS() {
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

//  This Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
   char c = GPS.read();
}

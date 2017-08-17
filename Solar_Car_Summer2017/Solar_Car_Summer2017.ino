#include <Wire.h>               //Accelerometer
#include <Adafruit_LSM303_U.h>  //Accelerometer
#include <Adafruit_Sensor.h>    //Accelerometer
#include <QTRSensors.h>         //QTR Sensors
#include <SoftwareSerial.h>     //Laser Range Finder

//------------------For Accelerometer------------------
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

//------------------For PING Ultrasonic Sensor------------------
const int pingSensor1 = 2;
const int pingSensor2 = 3;
const int pingSensor3 = 4;
const int pingSensor4 = 5;
int SensorNumber[4] = {pingSensor1, pingSensor2, pingSensor3, pingSensor4};
int SensArray[4] = {0, 0, 0, 0};


//------------------QTR Line Sensor------------------
//#define NUM_SENSORS   8     // number of sensors used
//#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
//#define EMITTER_PIN   2     // emitter is controlled by digital pin 2
//unsigned int sensorValues[NUM_SENSORS];
//QTRSensorsRC qtrrc((unsigned char[]) {30, 31, 32, 33, 34, 35, 36, 37},
//  NUM_SENSORS, TIMEOUT, EMITTER_PIN);
//boolean blackLineFlag=false;

//------------------H Bridge------------------
// motor one
int EnA = 6;
int Wheel1F = 7;
int Wheel1B = 8;
// motor two
int EnB = 11;
int Wheel2F = 9;
int Wheel2B = 10;

//Variables for 
int Speed = 150;
int Speed1 = 50;
int Speed2 = 50;
int counter = 0;
int dist = 0;

//------------------Laser Range Finder------------------
//#define rxPin    12  // Serial input (connects to the LRF's SOUT pin)
//#define txPin    13  // Serial output (connects to the LRF's SIN pin)
//
//#define BUFSIZE  16  // Size of buffer (in bytes) for incoming data from the LRF (this should be adjusted to be larger than the expected response)

// set up a new serial port
//SoftwareSerial lrfSerial =  SoftwareSerial(rxPin, txPin);

void setup() {
  //Initialize Serial Monitor 
  Serial.begin(9600);
  
//  #ifndef ESP8266
//    while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
//  #endif
  
  //------------------For Accelerometer------------------
  // Try to initialise and warn if we couldn't detect the LSM303 chip 
//  if (!accel.begin())
//  {
//    //Red wire = Power
//    //Black wire = Ground 
//    //Yellow Wire = SCL (Pin 21 on Arduino Mega) 
//    //Yellow Wire = SDA (Pin 20 on Arduino Mega) 
//    Serial.println("Oops ... unable to initialize the LSM303. Check your wiring!");
//    while (1);
//  }
  
//  attachInterrupt(0, interrupt, RISING);
  
  //------------------H Bridge------------------
  // set all the motor control pins to outputs
  pinMode(EnA, OUTPUT);
  pinMode(EnB, OUTPUT);
  pinMode(Wheel1F, OUTPUT);
  pinMode(Wheel1B, OUTPUT);
  pinMode(Wheel2F, OUTPUT);
  pinMode(Wheel2B, OUTPUT);

    // define pin modes
//  pinMode(rxPin, INPUT);
 // pinMode(txPin, OUTPUT);
  
  //-----------------Laser Range Finder------------------
//  while (!Serial);   // Wait until ready
//  Serial.println("\n\nParallax Laser Range Finder");
//  
//  // set the baud rate for the SoftwareSerial port
//  lrfSerial.begin(9600);
//
//  /*
//    When the LRF powers on, it launches an auto-baud routine to determine the
//    host's baud rate. it will stay in this state until a "U" ($55) character
//    is sent by the host. 
//  */
//  Serial.print("Waiting for the LRF...");
//  delay(2000);                        // Delay to let LRF module start up
//  lrfSerial.print('U');               // Send character
//  while (lrfSerial.read() != ':');    // When the LRF has initialized and is ready, it will send a single ':' character, so wait here until we receive it
//  delay(10);                          // Short delay
//  lrfSerial.flush();                  // Flush the receive buffer
//  Serial.println("Ready!");
//  Serial.flush();                     // Wait for all bytes to be transmitted to the Serial Monitor
}

void loop() {
  Forward();
  //------------------For Accelerometer------------------
 /* Get a new sensor event */
//  sensors_event_t event;
//  accel.getEvent(&event);

  /* Display the results (acceleration is measured in m/s^2) */
//  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
//  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
//  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
//  
  /* Delay before the next sample */
//  delay(500);

  
  //------------------For PING Ultrasonic Sensor------------------
  // establish variables for duration of the ping,
  // and the distance result in inches and centimeters:
//  long duration, inches, cm;
//
//  for(int s = 0; s < 4; s++){
//
//        // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
//    // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
//    pinMode(SensorNumber[s], OUTPUT);
//    digitalWrite(SensorNumber[s], LOW);
//    delayMicroseconds(2);
//    digitalWrite(SensorNumber[s], HIGH);
//    delayMicroseconds(5);
//    digitalWrite(SensorNumber[s], LOW);
//  
//    // The same pin is used to read the signal from the PING))): a HIGH
//    // pulse whose duration is the time (in microseconds) from the sending
//    // of the ping to the reception of its echo off of an object.
//    pinMode(SensorNumber[s], INPUT);
//    duration = pulseIn(SensorNumber[s], HIGH);
//  
//    // convert the time into a distance
//    inches = microsecondsToInches(duration);
//    cm = microsecondsToCentimeters(duration);
//    Serial.print("Sensor: ");
//    Serial.print(SensorNumber[s]);
//    Serial.print("   ");
//    Serial.print(inches);
//    Serial.print("in, ");
//    Serial.print(cm);
//    Serial.print("cm");
//    Serial.println();
//  }
//
//
//  delay(100);
  //------------------QTR Line Sensor------------------  
  // put your main code here, to run repeatedly:
  // prints sensor values 
//  qtrrc.read(sensorValues);
//
//  Serial.print(sensorValues[0]);
//  Serial.print("             ");
//  Serial.print(sensorValues[1]);
//  Serial.print("             ");
//
//  Serial.print(sensorValues[2]);
//  Serial.print("             ");
//
//  Serial.print(sensorValues[3]);
//  Serial.print("             ");
//
//  Serial.print(sensorValues[4]);
//  Serial.print("             ");
//
//  Serial.print(sensorValues[5]);
//  Serial.print("             ");
//
//  Serial.print(sensorValues[6]);
//  Serial.print("             ");
//
//  Serial.print(sensorValues[7]);
//  Serial.println("             ");

  //------------------Laser Range Finder------------------ 
  /* 
    When a single range (R) command is sent, the LRF returns the distance to the target
    object in ASCII in millimeters. For example:
     
    D = 0123 mm
  */   
//  lrfSerial.print('R');         // Send command
//  
//  // Get response back from LRF
//  // See Arduino readBytesUntil() as an alternative solution to read data from the LRF
//  char lrfData[BUFSIZE];  // Buffer for incoming data
//  char offset = 0;        // Offset into buffer
//  lrfData[0] = 0;         // Clear the buffer    
//  while(1)
//  {
//    if (lrfSerial.available() > 0) // If there are any bytes available to read, then the LRF must have responded
//    {
//      lrfData[offset] = lrfSerial.read();  // Get the byte and store it in our buffer
//      if (lrfData[offset] == ':')          // If a ":" character is received, all data has been sent and the LRF is ready to accept the next command
//      {
//        lrfData[offset] = 0; // Null terminate the string of bytes we just received
//        break;               // Break out of the loop
//      }
//          
//      offset++;  // Increment offset into array
//      if (offset >= BUFSIZE) offset = 0; // If the incoming data string is longer than our buffer, wrap around to avoid going out-of-bounds
//    }
//  }
//  Serial.println(lrfData);    // The lrfData string should now contain the data returned by the LRF, so display it on the Serial Monitor
//  Serial.flush();             // Wait for all bytes to be transmitted to the Serial Monitor
// 
}





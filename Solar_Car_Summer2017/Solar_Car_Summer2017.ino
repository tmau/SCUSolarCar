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

//------------------QTR Line Sensor------------------
#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2
unsigned int sensorValues[NUM_SENSORS];
QTRSensorsRC qtrrc((unsigned char[]) {30, 31, 32, 33, 34, 35, 36, 37},
  NUM_SENSORS, TIMEOUT, EMITTER_PIN);
boolean blackLineFlag=false;

//------------------H Bridge------------------
// motor one
int En1 = 9;
int Wheel1F = 10;
int Wheel1B = 11;
// motor two
int En2 = 6;
int Wheel2F = 7;
int Wheel2B = 8;

//Variables for 
int Speed = 0;
int Speed1 = 0;
int Speed2 = 0;
int counter = 0;
int dist = 0;

//------------------Laser Range Finder------------------
#define rxPin    12  // Serial input (connects to the LRF's SOUT pin)
#define txPin    13  // Serial output (connects to the LRF's SIN pin)
#define ledPin   14  // Most Arduino boards have an on-board LED on this pin

#define BUFSIZE  16  // Size of buffer (in bytes) for incoming data from the LRF (this should be adjusted to be larger than the expected response)

// set up a new serial port
SoftwareSerial lrfSerial =  SoftwareSerial(rxPin, txPin);

void setup() {
  //Initialize Serial Monitor 
  Serial.begin(9600);
  
  #ifndef ESP8266
    while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
  #endif
  
  //------------------For Accelerometer------------------
  // Try to initialise and warn if we couldn't detect the LSM303 chip 
  if (!accel.begin())
  {
    //Red wire = Power
    //Black wire = Ground 
    //Yellow Wire = SCL (Pin 21 on Arduino Mega) 
    //Yellow Wire = SDA (Pin 20 on Arduino Mega) 
    Serial.println("Oops ... unable to initialize the LSM303. Check your wiring!");
    while (1);
  }
  
//  attachInterrupt(0, interrupt, RISING);
  
  //------------------H Bridge------------------
  // set all the motor control pins to outputs
  pinMode(En1, OUTPUT);
  pinMode(En2, OUTPUT);
  pinMode(Wheel1F, OUTPUT);
  pinMode(Wheel1B, OUTPUT);
  pinMode(Wheel2F, OUTPUT);
  pinMode(Wheel2B, OUTPUT);

    // define pin modes
  pinMode(ledPin, OUTPUT);
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);

  digitalWrite(ledPin, LOW);  // turn LED off
  
  //-----------------Laser Range Finder------------------
  while (!Serial);   // Wait until ready
  Serial.println("\n\nParallax Laser Range Finder");
  
  // set the baud rate for the SoftwareSerial port
  lrfSerial.begin(9600);

  /*
    When the LRF powers on, it launches an auto-baud routine to determine the
    host's baud rate. it will stay in this state until a "U" ($55) character
    is sent by the host. 
  */
  Serial.print("Waiting for the LRF...");
  delay(2000);                        // Delay to let LRF module start up
  lrfSerial.print('U');               // Send character
  while (lrfSerial.read() != ':');    // When the LRF has initialized and is ready, it will send a single ':' character, so wait here until we receive it
  delay(10);                          // Short delay
  lrfSerial.flush();                  // Flush the receive buffer
  Serial.println("Ready!");
  Serial.flush();                     // Wait for all bytes to be transmitted to the Serial Monitor
}

void loop() {
  //------------------For Accelerometer------------------
 /* Get a new sensor event */
  sensors_event_t event;
  accel.getEvent(&event);

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
  
  /* Delay before the next sample */
  delay(500);

  
  //------------------For PING Ultrasonic Sensor------------------
  // establish variables for duration of the ping,
  // and the distance result in inches and centimeters:
  long duration, inches, cm;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingSensor1, OUTPUT);
  digitalWrite(pingSensor1, LOW);
  delayMicroseconds(2);
  digitalWrite(pingSensor1, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingSensor1, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pingSensor1, INPUT);
  duration = pulseIn(pingSensor1, HIGH);

  // convert the time into a distance
  inches = microsecondsToInches(duration);
  cm = microsecondsToCentimeters(duration);

  Serial.print(inches);
  Serial.print("in, ");
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();

  delay(100);
  //------------------QTR Line Sensor------------------  
  // put your main code here, to run repeatedly:
  // prints sensor values 
  qtrrc.read(sensorValues);

  Serial.print(sensorValues[0]);
  Serial.print("             ");
  Serial.print(sensorValues[1]);
  Serial.print("             ");

  Serial.print(sensorValues[2]);
  Serial.print("             ");

  Serial.print(sensorValues[3]);
  Serial.print("             ");

  Serial.print(sensorValues[4]);
  Serial.print("             ");

  Serial.print(sensorValues[5]);
  Serial.print("             ");

  Serial.print(sensorValues[6]);
  Serial.print("             ");

  Serial.print(sensorValues[7]);
  Serial.println("             ");

  //------------------Laser Range Finder------------------ 
  /* 
    When a single range (R) command is sent, the LRF returns the distance to the target
    object in ASCII in millimeters. For example:
     
    D = 0123 mm
  */   
  lrfSerial.print('R');         // Send command
  digitalWrite(ledPin, HIGH);   // Turn LED on while LRF is taking a measurement
  
  // Get response back from LRF
  // See Arduino readBytesUntil() as an alternative solution to read data from the LRF
  char lrfData[BUFSIZE];  // Buffer for incoming data
  char offset = 0;        // Offset into buffer
  lrfData[0] = 0;         // Clear the buffer    
  while(1)
  {
    if (lrfSerial.available() > 0) // If there are any bytes available to read, then the LRF must have responded
    {
      lrfData[offset] = lrfSerial.read();  // Get the byte and store it in our buffer
      if (lrfData[offset] == ':')          // If a ":" character is received, all data has been sent and the LRF is ready to accept the next command
      {
        lrfData[offset] = 0; // Null terminate the string of bytes we just received
        break;               // Break out of the loop
      }
          
      offset++;  // Increment offset into array
      if (offset >= BUFSIZE) offset = 0; // If the incoming data string is longer than our buffer, wrap around to avoid going out-of-bounds
    }
  }
  Serial.println(lrfData);    // The lrfData string should now contain the data returned by the LRF, so display it on the Serial Monitor
  Serial.flush();             // Wait for all bytes to be transmitted to the Serial Monitor
 
  digitalWrite(ledPin, LOW);  // Turn LED off
}

//------------------FUNTIONS BELOW------------------
//------------------For Accelerometer------------------
void displaySensorDetails(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}
//------------------For PING Ultrasonic Sensor------------------
long microsecondsToInches(long microseconds) {
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

//------------------H Bridge------------------
//Motor Control Functions 
void Forward()
{
  digitalWrite(Wheel1B, LOW);
  digitalWrite(Wheel2B, LOW);
  digitalWrite(Wheel1F, HIGH);
  digitalWrite(Wheel2F, HIGH);
  analogWrite(En1, Speed);
  analogWrite(En2, Speed);
}

void Forward2()
{
  digitalWrite(Wheel1B, LOW);
  digitalWrite(Wheel2B, LOW);
  digitalWrite(Wheel1F, HIGH);
  digitalWrite(Wheel2F, HIGH);
  analogWrite(En1, Speed1);
  analogWrite(En2, Speed2);
}

void Reverse()
{
  digitalWrite(En1, LOW);
  digitalWrite(En2, LOW);
  digitalWrite(Wheel1F, LOW);
  digitalWrite(Wheel2F, LOW); 
  digitalWrite(Wheel1B, HIGH);
  digitalWrite(Wheel2B, HIGH);
  analogWrite(En1, Speed);
  analogWrite(En2, Speed);
}

void Brake()
{
  digitalWrite(Wheel1F, LOW);
  digitalWrite(Wheel2F, LOW);
  digitalWrite(Wheel1B, LOW);
  digitalWrite(Wheel2B, LOW);
  analogWrite(En1, Speed);
  analogWrite(En2, Speed);
}

void Coast()
{

  digitalWrite(En1, LOW);
  digitalWrite(En2, LOW);
}

void PivotRight()
{
  //digitalWrite(En1, LOW);
  //digitalWrite(En2, LOW);
  digitalWrite(Wheel1B, LOW);
  digitalWrite(Wheel1F, LOW);
  digitalWrite(Wheel2B, LOW);
  digitalWrite(Wheel2F, HIGH);
  analogWrite(En2, Speed);
}

void PivotLeft()
{
  //digitalWrite(En1, LOW);
  //digitalWrite(En2, LOW);
  digitalWrite(Wheel1B, LOW);
  digitalWrite(Wheel2F, LOW);
  digitalWrite(Wheel2B, LOW);
  digitalWrite(Wheel1F, HIGH);
  analogWrite(En1, Speed);
}

void TurnLeft()
{
  digitalWrite(En1, LOW);
  digitalWrite(En2, LOW);
  digitalWrite(Wheel1B, LOW);
  digitalWrite(Wheel2F, LOW);
  digitalWrite(Wheel2B, HIGH);
  digitalWrite(Wheel1F, HIGH);
  analogWrite(En1, Speed);
  analogWrite(En2, Speed);
}

void TurnLeft2()
{
  digitalWrite(En1, LOW);
  digitalWrite(En2, LOW);
  digitalWrite(Wheel1B, LOW);
  digitalWrite(Wheel2F, LOW);
  digitalWrite(Wheel2B, HIGH);
  digitalWrite(Wheel1F, HIGH);
  analogWrite(En1, Speed1);
  analogWrite(En2, Speed2);
}

void TurnRight()
{
  digitalWrite(En1, LOW);
  digitalWrite(En2, LOW);
  digitalWrite(Wheel2B, LOW);
  digitalWrite(Wheel1F, LOW);
  digitalWrite(Wheel1B, HIGH);
  digitalWrite(Wheel2F, HIGH);
  analogWrite(En1, Speed);
  analogWrite(En2, Speed);
}

void TurnRight2()
{
  digitalWrite(En1, LOW);
  digitalWrite(En2, LOW);
  digitalWrite(Wheel2B, LOW);
  digitalWrite(Wheel1F, LOW);
  digitalWrite(Wheel1B, HIGH);
  digitalWrite(Wheel2F, HIGH);
  analogWrite(En1, Speed1);
  analogWrite(En2, Speed2);
}


void cmForward(int x)
{
   counter = 0;
   dist = ((360*x) /(60*PI))*10;//CHANGE
   Forward();
   while(counter < dist)
   {
    Serial.println(counter);
   }
   Brake();
}

void cmReverse(int x)
{
   counter = 0;
   dist = ((360*x) /(60*PI))*10; //CHANGE
   Reverse(); 
   while(counter > -dist)
   {
    Serial.println(counter);
   }
   Brake();
}

void KTurn()
{
  PivotLeft();
  delay(700);
  Brake();
  delay(500);
  TurnLeftReverse();
  delay(900);
  Brake();
  delay(500);
  Forward();
  delay(800);
  Brake();
  delay(500);
}

void TurnLeftReverse()
{
  //digitalWrite(En1, LOW);
  //digitalWrite(En2, LOW);
  digitalWrite(Wheel1B, LOW);
  digitalWrite(Wheel2F, LOW);
  digitalWrite(Wheel2B, HIGH);
  digitalWrite(Wheel1F, LOW);
  analogWrite(En1, Speed);
}



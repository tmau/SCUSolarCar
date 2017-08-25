#include <QTRSensors.h>
#include "moving_average.h"

//QTR Sensors -- for line following 
#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2
unsigned int sensorValues[NUM_SENSORS];
QTRSensorsRC qtrrc((unsigned char[]) {39, 41, 43, 45, 47, 49, 51, 53}, //Pins labeled 
  NUM_SENSORS, TIMEOUT, EMITTER_PIN);
boolean blackLineFlag=false;

 // PID GAINS: This is a method of controlling 
float KP = .1;
float KD = .3;

  int lastError = 0; 
  int pwmPeriod = 100; // PWM PERIOD
  int error_diff = 50; // ERROR BOUNDRIES
    

//Ping Sensor Pins
const int pingPin1 = 2; //Right Sensor
const int pingPin2 = 3; //Middle Sensor
const int pingPin3 = 4; //Left Sensor

// connect motor controller pins to Arduino digital pins
// motor one
int enA = 10;
int in1 = 9;
int in2 = 8;
//motor two 
int in3 = 7;
int in4 = 6;
int enB = 5;
int Speed = 150; 
int SpeedB = 255; //Go the maximum ammount you can turn

void Forward();
void Reverse(); 
void Left();
void Right(); 
void Brake();


void setup()
{
  Serial.begin(9600);
  // set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
}

void loop()
{
  ReadLine();
  //ReadLine();
//  ReadPing();
//  Left();
//  Forward();
  
//  //Backup to the left 
//  Left(); 
//  delay(2000);
//  Backward();
//  delay(2000);
//  Brake(); 
//  delay(2000);
//
//  //Forward to the right
//  Right(); 
//  delay(2000);
//  Forward(); 
//  delay(2000);
//  Brake(); 
//  delay(2000);
}
//Ping Sensor 
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

//Car Directions 
void Left(){
  analogWrite(enB, SpeedB);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);  
}
void Right(){
  analogWrite(enB, SpeedB);
  digitalWrite(in4, LOW);  
  digitalWrite(in3, HIGH);
}
void Backward()
{
  analogWrite(enA, Speed);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}
void Forward()
{
  analogWrite(enA, Speed);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}
void Brake()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enA, Speed);
  analogWrite(enB, Speed);
}
int TriggerRead(int pingPin){
  long duration, inches, cm;
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);
  inches = microsecondsToInches(duration);
  cm = microsecondsToCentimeters(duration);
  Serial.print(inches);
  Serial.print("in, ");
  Serial.print(cm);
  Serial.print("cm");
  Serial.print("             ");
  if (pingPin == pingPin3){
    Serial.println();
  }
  return inches; 
}
void ReadPing(){
  if (TriggerRead(pingPin2) < 30){
    Brake();
  }
  else if (TriggerRead(pingPin1) < 50){
    Left();
    Forward();
  }
  else if (TriggerRead(pingPin3) < 50){
    Brake(); 
    delay(2000);
    Left();
    delay(2000);
    Backward();
    delay(2000); 
    Brake();
    delay(2000);
    Forward(); 
    delay(2000); 
    Left(); 
    delay(2000);
    Forward();
    delay(2000);
 
//    Right();
//    Forward();
  }
  else{
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    Forward(); 
  }
}

void ReadLine(){
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
}

bool lineFollow()
{
    float pwmWidth;
    int error;
    unsigned int position = qtrrc.readLine(sensorValues);
    float distanceBack;
    
    if(position >= 7000)
      return false;
 
    Serial.println(sensorValues[3]); //For testing the middle consisting of sensors four and five
    Serial.println(sensorValues[4]);            
    
    // computes error
    error = position - 3500; // center of line is between 4 & 5 
              
    // PID CONTROL
    pwmWidth = abs(KP * error + KD * (error - lastError)); 
    lastError = error;     
    
    //need to update these so that they drive according to current programmed mechanics
    if(error > error_diff){
        Left();
        Forward();
        delay(9);
        delay(1);
    }
    
    else if(error < -1*error_diff){
        Right();
        Forward();
        delay(9);
        delay(1);
    }
    
    else{
        Forward();
        delay(9);
        delay(1);
    }
    return true;
    
}


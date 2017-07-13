// Lab 8
#include <QTRSensors.h>

//QTR Sensors
#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2
unsigned int sensorValues[NUM_SENSORS];
QTRSensorsRC qtrrc((unsigned char[]) {30, 31, 32, 33, 34, 35, 36, 37},
  NUM_SENSORS, TIMEOUT, EMITTER_PIN);
boolean blackLineFlag=false;


void setup() {
  


  Serial.begin(9600);
}
void loop() {
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
}




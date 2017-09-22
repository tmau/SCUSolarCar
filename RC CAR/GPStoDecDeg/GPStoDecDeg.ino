#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <math.h>                                 // used by: GPS


SoftwareSerial gpsSerial(11, 12); // RX, TX (TX not used)
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);


const int sentenceSize = 80;
double aveValue;
double intLat;
double intLong;
double sumLat;
double sumLong;
double aveLat;
double aveLong;
float currentLat, currentLong; 
int countLat;
int countLong;
char sentence[sentenceSize];

void setup()
{
  Serial.begin(9600);
  gpsSerial.begin(9600);
  
  sumLat=0;
  sumLong=0;
  aveValue=1; 
  countLat=0;
  countLong=0;

  
}

void loop()
{
  static int i = 0;
  if (gpsSerial.available())
  {
    char ch = gpsSerial.read();
    if (ch != '\n' && i < sentenceSize)
    {
      sentence[i] = ch;
      i++;
    }
    else
    {
     sentence[i] = '\0';
     i = 0;
     displayGPS();
    }
  }
}

void displayGPS()
{
  
  sensors_event_t event; 
  mag.getEvent(&event);
  float Pi = 3.14159;
  // Calculate the angle of the vector y,x
  float heading = (atan2(event.magnetic.y,event.magnetic.x) * 180) / Pi;
  
  char field[20];
  getField(field, 0);

  if (heading < 0)
  {
    heading = 360 + heading;
  }

  
  if (strcmp(field, "$GPRMC") == 0)
  {
    //Serial.print("Lat: ");
    getField(field, 3);  // number
    intLat=atof(field);
    //Serial.print(intLat,5);
    //Serial.println(field);
    if (countLat<aveValue){
      sumLat=intLat+sumLat;
      countLat++;
    }
    else{
      aveLat=sumLat/aveValue;
      Serial.print("Latitude: ");
      Serial.println(aveLat,5);
      currentLat = convertDegMinToDecDeg(sumLat);
      Serial.print("Current Lat: "); 
      Serial.println(currentLat,5);

      countLat=0;
      sumLat=0;
    }

    
    getField(field, 4); // N/S

    //Serial.print(field);
    
    //Serial.print(" Long: ");
    getField(field, 5);  // number
    //Serial.println(",");
    intLong=atof(field);
    if (countLong<aveValue){
      sumLong=intLong+sumLong;
      countLong++;
    }
    else{
      aveLong=sumLong/aveValue;
      Serial.print("Longitude: ");
      Serial.println(aveLong,6);
      currentLong = convertDegMinToDecDeg(aveLong); 
      Serial.print("CURRENT LONG: ");
      Serial.println(currentLong,5);
      //Serial.println(" ");
      countLong=0;
      sumLong=0;
    }
    Serial.println(".");
    //Serial.print(intLong,5);
    //Serial.println(field);
    

    //Serial.print(field);
    getField(field, 6);  // E/W
    //Serial.println(field);
  }
}

void getField(char* buffer, int index)
{
  int sentencePos = 0;
  int fieldPos = 0;
  int commaCount = 0;
  while (sentencePos < sentenceSize)
  {
    if (sentence[sentencePos] == ',')
    {
      commaCount ++;
      sentencePos ++;
    }
    if (commaCount == index)
    {
      buffer[fieldPos] = sentence[sentencePos];
      fieldPos ++;
    }
    sentencePos ++;
  }
  buffer[fieldPos] = '\0';
} 


// converts lat/long from Adafruit degree-minute format to decimal-degrees; requires <math.h> library
double convertDegMinToDecDeg (float degMin) 
{
  double min = 0.0;
  double decDeg = 0.0;
 
  //get the minutes, fmod() requires double
  min = fmod((double)degMin, 100.0);
 
  //rebuild coordinates in decimal degrees
  degMin = (int) ( degMin / 100 );
  decDeg = degMin + ( min / 60 );
 
  return decDeg;
}


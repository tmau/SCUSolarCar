#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <math.h>                                 // used by: GPS

LiquidCrystal_I2C lcd(0x3F,20,4);  // set the LCD address to 0x3F for a 20 chars and 4 line display
SoftwareSerial gpsSerial(11, 12); // RX, TX (TX not used)
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);


const int sentenceSize = 80;
float currentLong;
float currentLat = 0; 
char sentence[sentenceSize];

// connect motor controller pins to Arduino digital pins
// motor one
int enA = 10;
int in1 = 9;
int in2 = 8;
//motor two 
int in3 = 7;
int in4 = 6;
int enB = 5;
int Speed = 200; 
int SpeedB = 255; //Go the maximum ammount you can turn

void Forward();
void Reverse(); 
void Left();
void Right(); 
void Brake();

void setup()
{
  Serial.begin(9600);
  gpsSerial.begin(9600);
  
  lcd.init();                      // initialize the lcd   
  lcd.backlight();

  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    lcd.setCursor(0,3);
    lcd.print("No LSM303 detected");
    return;
  }
  for(int i=10; i > 0 ; i--){
    lcd.setCursor(10,0); 
    lcd.print(i);
    delay(1000);
    lcd.clear();
  }

  lcd.setCursor(0,0);
  lcd.print("Tracking GPS...");
  lcd.clear();
  
  while(currentLat== 0){
    updateGPS();
  }
  
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
  //Part 1
  Forward(); 
  delay(5000); 
  
  updateGPS();
  
  Brake(); 
  delay(2000);
   
  updateGPS(); 
  
  Left();
  Forward(); 
  delay(1750); 

  updateGPS();

  Brake(); 
  delay(2000); 
  
  updateGPS();
  
  //Part 2
  Forward(); 
  delay(3000); 
  
  updateGPS();
  
  Brake(); 
  delay(2000);
  
  updateGPS(); 

  Left(); 
  Forward();
  delay(1750); 

  updateGPS();

  Brake(); 
  delay(2000); 

  updateGPS();
}
void updateGPS()
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
}//update GPS


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
    getField(field, 3);  // number
    currentLat=atof(field);
    currentLat = convertDegMinToDecDeg(currentLat);
    getField(field, 4); // N/S
    getField(field, 5);  // number
    currentLong=atof(field);
    currentLong = convertDegMinToDecDeg(currentLong); 
    getField(field, 6);  // E/W
    //Serial Printing
//    Serial.print("CURRENT LAT: "); 
//    Serial.println(currentLat,5);
//    Serial.print("CURRENT LONG: ");
//    Serial.println(currentLong,5);
//    Serial.print("HEADING: ");
//    Serial.println(heading);
//    Serial.println(".");
    //LCD Printing
    lcd.setCursor(0,0);
    lcd.print("Latitude: ");
    lcd.print(currentLat,3);
    lcd.setCursor(0,1);
    lcd.print("Longitude: ");
    lcd.print(currentLong,3);
    lcd.setCursor(0,3);
    lcd.print(heading);
  }
}

//For GPS
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

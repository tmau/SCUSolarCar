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
double intLat;
double intLong;
float currentLat, currentLong; 
char sentence[sentenceSize];

void setup()
{
  Serial.begin(9600);
  gpsSerial.begin(9600);
  
  lcd.init();                      // initialize the lcd   
  lcd.backlight();

  lcd.setCursor(0,0);
  lcd.print("Tracking GPS...");

  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    lcd.setCursor(0,3);
    lcd.print("No LSM303 detected");
  }
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
    getField(field, 3);  // number
    currentLat=atof(field);
    currentLat = convertDegMinToDecDeg(currentLat);
    getField(field, 4); // N/S
    getField(field, 5);  // number
    currentLong=atof(field);
    currentLong = convertDegMinToDecDeg(currentLong); 
    getField(field, 6);  // E/W
    //Serial Printing
    Serial.print("CURRENT LAT: "); 
    Serial.println(currentLat,5);
    Serial.print("CURRENT LONG: ");
    Serial.println(currentLong,5);
    Serial.print("HEADING: ");
    Serial.println(heading);
    Serial.println(".");
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


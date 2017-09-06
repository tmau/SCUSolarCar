/* Work on this one. Add Latitudes and Longitudes and average them
 *  
 *  
 *  
 * 
 */

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>


LiquidCrystal_I2C lcd(0x3F,20,4);  // set the LCD address to 0x3F for a 20 chars and 4 line display
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
int countLat;
int countLong;
char sentence[sentenceSize];

void setup()
{
  Serial.begin(9600);
  gpsSerial.begin(9600);
  
  sumLat=0;
  sumLong=0;
  aveValue=5; 
  countLat=0;
  countLong=0;
  
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
  lcd.setCursor(0,3);
  //lcd.print("Heading: ");
  lcd.print(heading);

  
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
      //lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Latitude: ");
      lcd.print(aveLat,3);
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
      Serial.println(aveLong,5);
      lcd.setCursor(0,1);
      lcd.print("Longitude: ");
      lcd.print(aveLong,3);
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


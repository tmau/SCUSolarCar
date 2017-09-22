// LCD Display
LiquidCrystal_I2C lcd(0x3F, 20, 4);  // Set the LCD I2C address and size (4x20)
#define LEFT_ARROW 0x7F
#define RIGHT_ARROW 0x7E
#define DEGREE_SYMBOL 0xDF

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

void updateDisplay(void)
{

  static unsigned long lastUpdate = millis();       // for controlling frequency of LCD updates
  unsigned long currentTime;

  // check time since last update
  currentTime = millis();
  if (lastUpdate > currentTime)   // check for time wrap around
    lastUpdate = currentTime;      

  if (currentTime >= lastUpdate + 500 )   // limit refresh rate
  {
    lastUpdate = currentTime;

    // line 1
    lcd.clear();
    lcd.print(F("tH= "));
    lcd.print(targetHeading, DEC);
    lcd.write(DEGREE_SYMBOL);
    lcd.print(F(" cH= "));
    lcd.print(currentHeading, DEC);
    lcd.write(DEGREE_SYMBOL);
  
    // line 2
    lcd.setCursor(0, 1);
    lcd.print(F("Err "));
    if (headingError < 0)
      lcd.write(LEFT_ARROW);
    lcd.print(abs(headingError), DEC);
    if (headingError > 0)
      lcd.write(RIGHT_ARROW);
    lcd.print(F(" Dist "));
    lcd.print(distanceToTarget, DEC);
    lcd.print(F("m "));
    #ifdef USE_GRAPHING
      lcd.write(map(distanceToTarget, 0, originalDistanceToTarget, 0, 7));    // show tiny bar graph of distance remaining
    #endif
    
    // line 3
    lcd.setCursor(0, 2);
    lcd.print(F("Snr "));
    lcd.print(sonarDistance, DEC);
    #ifdef USE_GRAPHING
      lcd.write(map(sonarDistance, 0, MAX_DISTANCE_IN, 0, 7));
    #endif
    lcd.print(F(" Spd "));
    lcd.print(speed, DEC);
    #ifdef USE_GRAPHING
      lcd.write(map(speed, 0, 255, 0, 7));
    #endif
  
    // line 4
    lcd.setCursor(0, 3);
    lcd.print(F("Mem "));
    lcd.print(freeRam(), DEC);
    lcd.print(F(" WPT "));
    lcd.print(waypointNumber + 1, DEC);
    lcd.print(F(" OF "));
    lcd.print(NUMBER_WAYPOINTS - 1, DEC);


     #ifdef DEBUG
      //Serial.print("GPS Fix:");
      //Serial.println((int)GPS.fix);
      Serial.print(F("LAT = "));
      Serial.print(currentLat);
      Serial.print(F(" LON = "));
      Serial.println(currentLong);
      //Serial.print("Waypint LAT ="); 
      //Serial.print(waypointList[waypointNumber].getLat());
      //Serial.print(F(" Long = "));
      //Serial.print(waypointList[waypointNumber].getLong());
      Serial.print(F(" Dist "));
      Serial.print(distanceToWaypoint());
      Serial.print(F("Original Dist "));
      Serial.println(originalDistanceToTarget);
      Serial.print(F("Compass Heading "));
      Serial.println(currentHeading);
      Serial.print(F("GPS Heading "));
      Serial.println(GPS.angle);
      
      //Serial.println(GPS.lastNMEA());
     
      //Serial.print(F("Sonar = "));
      //Serial.print(sonarDistance, DEC);
      //Serial.print(F(" Spd = "));
      //Serial.println(speed, DEC);
      //Serial.print(F("  Target = "));
      //Serial.print(targetHeading, DEC);
      //Serial.print(F("  Current = "));
      //Serial.print(currentHeading, DEC);
      //Serial.print(F("  Error = "));
      //Serial.println(headingError, DEC);
      //Serial.print(F("Free Memory: "));
      //Serial.println(freeRam(), DEC);
     #endif

  } //  if (currentTime >= lastUpdate + 500 )

}  // updateDisplay()  

// Graphing (mini-inline bar graph for LCD display)
#ifdef USE_GRAPHING
  void createLCDChars(void)
    {
    int lvl = 0;
    byte arry[8];
    for (int a = 7; a >= 0; a--)
      {
      for (int b = 0; b <= 7; b++)
      {
          if (b >= lvl)
            arry[b] = B11111;       // solid
          else
            //arry[b] = B00000;     // blank row
            arry[b] = B10001;       // hollow but with sides
       }
      lcd.createChar(a, arry);
      lvl++;
    }
  } // createLCDChars(void)
#endif








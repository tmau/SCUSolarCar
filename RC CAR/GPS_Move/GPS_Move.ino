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


// Compass navigation
int targetHeading;              // where we want to go to reach current waypoint
int currentHeading;             // where we are actually facing now
int headingError;               // signed (+/-) difference between targetHeading and currentHeading
#define HEADING_TOLERANCE 5     // tolerance +/- (in degrees) within which we don't attempt to turn to intercept targetHeading

//Waypoint navigation 
float currentLat,
      currentLong;
float targetLat = 3720.940;
float targetLong = 12156.296;
int distanceToTarget,            // current distance to target (current waypoint)
    originalDistanceToTarget;    // distance to original waypoing when we started navigating to it
#define WAYPOINT_DIST_TOLERANE  5   // tolerance in meters to waypoint; once within this tolerance, will advance to the next waypoint

#define TURN_LEFT 1
#define TURN_RIGHT 2
#define TURN_STRAIGHT 99

// Steering/turning 
enum directions {left = TURN_LEFT, right = TURN_RIGHT, straight = TURN_STRAIGHT} ;
directions turnDirection = straight;

// Object avoidance distances (in inches)
#define SAFE_DISTANCE 70
#define TURN_DISTANCE 40
#define STOP_DISTANCE 12

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
  //Setup pins for the motor 
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  //Setup the GPS 
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
  courseToWaypoint();
  moveCar();
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
      //currentLat = aveLat; 
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
      //currentLong = aveLong;
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

int distanceToWaypoint() 
{
  
  float delta = radians(currentLong - targetLong);
  float sdlong = sin(delta);
  float cdlong = cos(delta);
  float lat1 = radians(currentLat);
  float lat2 = radians(targetLat);
  float slat1 = sin(lat1);
  float clat1 = cos(lat1);
  float slat2 = sin(lat2);
  float clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong); 
  delta = sq(delta); 
  delta += sq(clat2 * sdlong); 
  delta = sqrt(delta); 
  float denom = (slat1 * slat2) + (clat1 * clat2 * cdlong); 
  delta = atan2(delta, denom); 
  distanceToTarget =  delta * 6372795; 
   
  // check to see if we have reached the current waypoint
  if (distanceToTarget <= WAYPOINT_DIST_TOLERANE)
    //nextWaypoint(); //CHANGE 
    Serial.print("MADE IT");
    
  return distanceToTarget;
}  // distanceToWaypoint()


void calcDesiredTurn(void)
{
    // calculate where we need to turn to head to destination
    headingError = targetHeading - currentHeading;
    
    // adjust for compass wrap
    if (headingError < -180)      
      headingError += 360;
    if (headingError > 180)
      headingError -= 360;
  
    // calculate which way to turn to intercept the targetHeading
    if (abs(headingError) <= HEADING_TOLERANCE)      // if within tolerance, don't turn
      turnDirection = straight;  
    else if (headingError < 0)
      turnDirection = left;
    else if (headingError > 0)
      turnDirection = right;
    else
      turnDirection = straight;
 
}  // calcDesiredTurn()


int courseToWaypoint() 
{
  float dlon = radians(targetLong-currentLong);
  float cLat = radians(currentLat);
  float tLat = radians(targetLat);
  float a1 = sin(dlon) * cos(tLat);
  float a2 = sin(cLat) * cos(tLat) * cos(dlon);
  a2 = cos(cLat) * sin(tLat) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  targetHeading = degrees(a2);
  return targetHeading;
}   // courseToWaypoint()

void moveCar(){
  if (turnDirection == straight){
    Forward();
    delay(500); 
  }
  if (turnDirection == left){
    Left(); 
    Forward();
    delay(500); 
  }
  if (turnDirection == right){
    Right();
    Forward();
    delay(500); 
  }
  return;
  
}

//void moveAndAvoid(void)
//{
//
//    if (sonarDistance >= SAFE_DISTANCE)       // no close objects in front of car
//        {
//           if (turnDirection == straight)
//             speed = FAST_SPEED;
//           else
//             speed = TURN_SPEED;
//           driveMotor->setSpeed(speed);
//           driveMotor->run(FORWARD);       
//           turnMotor->run(turnDirection);
//           return;
//        }
//      
//     if (sonarDistance > TURN_DISTANCE && sonarDistance < SAFE_DISTANCE)    // not yet time to turn, but slow down
//       {
//         if (turnDirection == straight)
//           speed = NORMAL_SPEED;
//         else
//           {
//              speed = TURN_SPEED;
//              turnMotor->run(turnDirection);      // alraedy turning to navigate
//            }
//         driveMotor->setSpeed(speed);
//         driveMotor->run(FORWARD);       
//         return;
//       }
//     
//     if (sonarDistance <  TURN_DISTANCE && sonarDistance > STOP_DISTANCE)  // getting close, time to turn to avoid object        
//        {
//          speed = SLOW_SPEED;
//          driveMotor->setSpeed(speed);      // slow down
//          driveMotor->run(FORWARD); 
//          switch (turnDirection)
//          {
//            case straight:                  // going straight currently, so start new turn
//              {
//                if (headingError <= 0)
//                  turnDirection = left;
//                else
//                  turnDirection = right;
//                turnMotor->run(turnDirection);  // turn in the new direction
//                break;
//              }
//            case left:                         // if already turning left, try right
//              {
//                turnMotor->run(TURN_RIGHT);    
//                break;  
//              }
//            case right:                       // if already turning right, try left
//              {
//                turnMotor->run(TURN_LEFT);
//                break;
//              }
//          } // end SWITCH
//          
//         return;
//        }  
//
//
//     if (sonarDistance <  STOP_DISTANCE)          // too close, stop and back up
//       {
//         driveMotor->run(RELEASE);            // stop 
//         turnMotor->run(RELEASE);             // straighten up
//         turnDirection = straight;
//         driveMotor->setSpeed(NORMAL_SPEED);  // go back at higher speet
//         driveMotor->run(BACKWARD);           
//         while (sonarDistance < TURN_DISTANCE)       // backup until we get safe clearance
//           {
//              if(GPS.parse(GPS.lastNMEA()) )
//                 processGPS();  
//              currentHeading = readCompass();    // get our current heading
//              calcDesiredTurn();                // calculate how we would optimatally turn, without regard to obstacles      
//              checkSonar();
//              updateDisplay();
//              delay(100);
//           } // while (sonarDistance < TURN_DISTANCE)
//         driveMotor->run(RELEASE);        // stop backing up
//         return;
//        } // end of IF TOO CLOSE
//     
//}   // moveAndAvoid()

//Motor code for driving the car
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

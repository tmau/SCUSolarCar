
//libraries needed
#include <Servo.h>
#include <stdbool.h>
#include <StackArray.h>
#include <TinyGPS.h>
#include <QTRSensors.h>
#include <Wire.h>
#include <LSM303.h>

#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds
#define EMITTER_PIN   QTR_NO_EMITTER_PIN     // no emitter

/* ====================================================================================================== */
//The name for the servo. Happens to be teamTKE here
//and all data structures used.
Servo teamTKE;
LSM303 compass;
TinyGPS gps;
StackArray<float> lati;
StackArray<float> longi;

/* ====================================================================================================== */
//initial position of the servo. in terms of degrees
int pos = 73;

/* ====================================================================================================== */
//sensor initializations
int IN1 = 33;
int IN2 = 32;
int pingPin = 35; //left ---- 
int pingPin2 = 36; //right
int pingPin3 = 37;//outer left
int pingPin4 = 38;//outer Right

//constants -- #define's did funny things to arduino compiler so used consts
const float v = 331.5 + 0.6*20;
const int maxTurnAngle = 22;
const int THRESHOLD = 30;
const int lowSpeed = 160;
const int topSpeed = 220;
const float declinationAngle1 = 13.5;
//const int num_obstacles = 4;

//global variables -- sorry to anyone who is reading this, did not want to deal with mass amounts of pointers. 
int Speed = 0;
char dir = 'n';
char state = 'b';
int obstacle_counter = 0;

//-----------line follow globals-------------------------
  int i, least, greatest; //For the PID control

  QTRSensorsRC qtrrc((unsigned char[]) {46, 47, 48, 49, 50, 51, 52, 53}, NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
  unsigned int sensors[8];
  
  unsigned int sensorValues[NUM_SENSORS];
  int lastError = 0; 
  int pwmPeriod = 100; // PWM PERIOD
  int error_diff = 50; // ERROR BOUNDRIES
    
  // PID GAINS: This is a method of controlling 
  float KP = .1;
  float KD = .3;
    

/* ====================================================================================================== */
//void setup. Runs once
void setup() 
{  
  Serial.begin(115200);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  //master slave setup
  teamTKE.attach(31);
  teamTKE.write(pos);
    
  Serial.println("---------------------START----------------------------------------------------");
  Serial3.print("$PMTK313,1*2E\r\n");  // Enable to search a SBAS satellite
  Serial1.print("$PMTK301,2*2E\r\n");  // Enable WAAS as DGPS Source
  Serial1.print("$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"); //causes GPS only to give out GPGGA information
  Serial1.print("$PMTK220,200*2C\r\n"); //increases rate of GPS from 1Hz to 5Hz
  
  createDatabase();
  
    //output from the photoresistor 4-11 corresponds with photoresistors 1-8
  pinMode(46, OUTPUT);
  pinMode(47, OUTPUT);
  pinMode(48, OUTPUT);
  pinMode(49, OUTPUT);
  
  pinMode(50, OUTPUT);
  pinMode(51, OUTPUT);
  pinMode(52, OUTPUT);
  pinMode(53, OUTPUT);
  
  //H-bridge interaction
  pinMode(3, INPUT);  //IN1
  pinMode(5, INPUT);  //IN2

  
  Serial.println("Will Calibrate for 15ish Seconds");
       
  //CALIBRATION FOR PHOTORESISTORS
  delay(500); 
  pinMode(13, OUTPUT); 
  digitalWrite(13, HIGH);     
  for (int i = 0; i < 400; i++){  // 10 Second Calibration
     qtrrc.calibrate();       // Reads all sensors 10 times 
  }
  
  //Now that the calibration is done we can move on with the actual code  
  digitalWrite(13, LOW); 
  Serial.println("calibration complete");
  
  
  //compass callibration and initialization
  Wire.begin();
  compass.init();
  compass.enableDefault();
  
  /*
  Calibration values; the default values of +/-32767 for each axis
  lead to an assumed magnetometer bias of 0. Use the Calibrate example
  program to determine appropriate values for your particular unit.
  */
  compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
  compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};

}


/* ====================================================================================================== */
//This code runs repeatedly
void loop() {
  
  //work the ping sensors
  int left, right, outerLeft, outerRight;
  pingSense(&left, &right, &outerLeft, &outerRight);
  printPings(left, right, outerLeft, outerRight);
  
    if(state == 'b')
    {
      //initial case, when car is starting out of house
      //backs up and gets a gps heading
      Backward(topSpeed);
      delay(7000);
      Stop();
      accelerate(Speed, topSpeed);
      state = 'a';
    }
    else if(state == 'a')
    {
     //arbiter. does 10 ping senses before getting a new gps heading. 
    //this should amount to about a gps reading every second and 10 ping senses every second
        for(int i=0; i < 10; i++)
        {
         pingSense(&left, &right, &outerLeft, &outerRight);
         printPings(left, right, outerLeft, outerRight);
         delay(20);
         if(checkPings(left, right, outerLeft, outerRight))
         {
            state = 's';
            obstacle_counter++;
            break;
         }
         else
           state = 'g';
           //Serial.println("gps heading");
        } 
    }
    else if(state == 's')
    {
      //sensor code. switches depending on the direction it needs to turn - direction given by checkPings(), which updates a global 'dir'
      // 'b' is backwards
      // 'r' is right turn event
      // 'l' is left turn event 
      switch(dir) {
      /********************************************************************/ 
    case 'b' :
            {
            //two front triggers hit. 
                Stop();
                delay(200);
                teamTKE.write(pos + maxTurnAngle);
                Backward(200);
                delay(2000);
                Stop();
                teamTKE.write(pos);
              accelerate(Speed, topSpeed);
                break;
              }
        /********************************************************************/ 
         case 'l' :
              {
                Serial.println("INSIDE SENSOR RANGE!!!");
                int data = (maxTurnAngle + 5) - (maxTurnAngle * (outerRight / THRESHOLD));
                //data = 10;
                data = min(data, maxTurnAngle); 
                teamTKE.write(pos - data);
                decelerate(Speed, lowSpeed);
                delay(300);
                teamTKE.write(pos + data);
                delay(300);
                teamTKE.write(pos);
                accelerate(Speed, topSpeed);
                break;
              }
       /********************************************************************/  
         case 'r' :
              {
                Serial.println("INSIDE!!!");
                int data = (maxTurnAngle + 5) - (maxTurnAngle * (outerLeft / THRESHOLD));
                //data = 10;
                data = min(data, maxTurnAngle);
                teamTKE.write(pos + data);
                decelerate(topSpeed, lowSpeed);
                delay(300);
                teamTKE.write(pos - data);
                delay(300); 
                teamTKE.write(pos);
                accelerate(lowSpeed, topSpeed);
                break;
              }
      /********************************************************************/ 
         default :
         { 
                Forward(topSpeed);
                break;
         }
      }
      state = 'a';
    }
    else if(state == 'l')
    {
      //line follow mode. follows a black line on a white surface. 'straight' is when the line is between
      //photoresistors 3 and 4. if all 8 photoresistors are triggered, stops the loop and proceeds to 'e'
      accelerate(Speed, topSpeed);
      while(lineFollow());
      state = 'e';
    }
    else if(state == 'e')
    {
      //end state, stops car and launches a permanent while loop to keep
      //the car still. Can also delay for a full charge period. 
      int fullChargePeriod = 100000000; //should this be a define/global?
      Stop();
      Serial.println("end");
      delay(fullChargePeriod);
      state = 'l';
    }
    
    else if(state == 'g')
    {
      //gps mode - gets the current position using the gps and TinyGPS library.
      //gets current heading from LSM303 compass
      //calculates desired heading using course_to() function
      //finds a turn angle by subracting the current heading from the desired heading and doing a bit
      // of correction. 
      while (Serial3.available()) 
      { 
        compass.read();
       // Serial.println("compass read");
       // unsigned char cc = Serial3.read(); //read from gps
        if (gps.encode(Serial3.read())) 
        {
          float flat, flon;
          unsigned long age;
          gps.f_get_position(&flat, &flon, &age);
          Serial.print("Lat: ");
          Serial.print(flat);
          Serial.print(", Long: ");
          Serial.println(flon);
          
          float newDirection = desiredHeading(flat, flon);
          if(checkwp(flat, flon))
            newDirection = desiredHeading(flat, flon);
          float currDirection = compass.heading();  //find current heading from compass
          float turnangle = newDirection-currDirection; //error in heading between desired and current
          
          int c = 0;
          //while loop to make car go straight if the current heading is within +/- 5degrees of desired or taken 5 calculations
          while (newDirection > currDirection + 5 || newDirection < currDirection - 5 && c < 5)
          {
            if (turnangle >= 180)
              turnangle -= 360;
            if (turnangle <= -180)
              turnangle += 360;
            if (turnangle > maxTurnAngle)
              turnangle = maxTurnAngle;//if the necessary turn angle is greater than the max turn angle of wheels, set turn angle to max turn angle
            if (turnangle < -maxTurnAngle)
              turnangle = -maxTurnAngle;
            
            Serial.print("turnangle calculated: ");
            Serial.println(turnangle);
            
            compass.read();           
            gps.f_get_position(&flat, &flon, &age);
            newDirection = desiredHeading(flat, flon);
            if(checkwp(flat, flon))
                newDirection = desiredHeading(flat, flon);
            currDirection = compass.heading();  //find current heading from compass
            turnangle = newDirection-currDirection; //error in heading between desired and current
            c++;
           }
          teamTKE.write(pos + turnangle);
        }
      }
      
      //get lati and longi stack counts. if tehy are both 0, go to 'l'
      if(lati.isEmpty() && longi.isEmpty())
        state = 'l'; 
    }
} 

/* ====================================================================================================== */
//checks all sensors if they are inside threshold and updates direction accordingly
//returns true if a ping is inside the threshold, false else
//dir is updated to 'n' if there is no direction to turn
bool checkPings(int left, int right, int outerLeft, int outerRight)
{
  if(right < THRESHOLD && left < THRESHOLD)
  {
    dir = 'b';
    return true;
  }
  else if(outerLeft < THRESHOLD)
  {
    dir = 'r';
    return true;
  }  
  else if(outerRight < THRESHOLD)
  {
    dir = 'l';
    return true;
  }
  
    dir = 'n';
  return false;
}  

/* ====================================================================================================== */
//prints the current values of the ping sensors
void printPings(int left, int right, int outerLeft, int outerRight)
{
  Serial.print("Outer Right --------   ");
  Serial.println(outerRight, DEC);
  delay(10);
  Serial.print("Inner Right -----------------  ");
  Serial.println(right, DEC);
  delay(10);
  Serial.print("Inner left ----------------------------    ");
  Serial.println(left, DEC);
  delay(10); //delays the loop a bit
  Serial.print("Outer Left ------------------------------------------   ");
  Serial.println(outerLeft, DEC);
  delay(10);
 }
 
/* ====================================================================================================== */
//gets all the ping sensors values
static void pingSense(int *left, int *right, int *outerLeft, int *outerRight)
{
 *outerRight = distanceCm(pingPin);
 delay(10);
 *left = distanceCm(pingPin2);
 delay(10);
 *right = distanceCm(pingPin3);
 delay(10);
 *outerLeft = distanceCm(pingPin4);
 
  delay(100); 
}
   
/* ====================================================================================================== */
//Ping code for pingPin
//returns the distance from the ping sensor an object is in centimeters
float distanceCm(int sensor) 
{
  //sending the sound pulse
  pinMode(sensor, OUTPUT); //ping uses same pin for input and output
  digitalWrite(sensor, LOW);
  delayMicroseconds(3); //wait for the pin to settle
  digitalWrite(sensor, HIGH);
  delayMicroseconds(5); 
  digitalWrite(sensor, LOW);
  
  //listen for echo
  pinMode(sensor, INPUT);
  float tUs = pulseIn(sensor, HIGH); //measures how long it takes for pingpin to get to low
  float t = tUs / 1000.0 / 1000.0 / 2;
  float d = t*v; //distance = time times speed
  return d * 100;
} 

/* ====================================================================================================== */
//makes wheels run forward
void Forward (int Speed) {

  //analogWrite (IN1, 0);
  //analogWrite (IN2, Speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

}

/* ====================================================================================================== */
//makes wheels go backward
void Backward (int Speed) {

  //analogWrite (IN1, Speed);
  //analogWrite (IN2, 0);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

}

/* ====================================================================================================== */
//makes wheels stop
void Stop () {
  
  //analogWrite (IN1, 0);
  //analogWrite (IN2, 0);
  digitalWrite(32, LOW);
  digitalWrite(33, LOW);
}

/* ====================================================================================================== */
//accelerates car from currSpeed to newSpeed
void accelerate(int currSpeed, int newSpeed)
{
  digitalWrite(IN2, HIGH);
  /*
  for(int i = currSpeed; i < newSpeed; i+=10)
  {
    Forward(i);
    delay(20);
  }
  Speed = newSpeed;
  */
}
/* ====================================================================================================== */
//decelerates car from currSpeed to newSpeed
void decelerate(int currSpeed, int newSpeed)
{
  /*
  for(int i = currSpeed; i < newSpeed; i+=10)
  {
    Forward(i);
    delay(20);
  }
  Speed = newSpeed;
  */
}


/* ====================================================================================================== */
//this is the gps code. reads the compass for current heading. then calculates 
//desired heading. then turns car accordingly. loops through that five times before returning
//to arbiter
/*
void turnCar()
{
   while (Serial1.available()) 
      { 
        compass.read();
        unsigned char cc = Serial1.read(); //read from gps
        if (gps.encode(cc)) 
        {
          float flat, flon;
          //flat = gps.location.lat();
          //flon = gps.location.lng();
          gps.get_
          
          Serial.print("Lat: ");
          Serial.print(flat);
          Serial.print(", Long: ");
          Serial.println(flon);
          
          float newDirection = desiredHeading(flat, flon);
          if(checkwp(flat, flon))
            newDirection = desiredHeading(flat, flon);
          float currDirection = compass.heading();  //find current heading from compass
          float turnangle = newDirection-currDirection; //error in heading between desired and current
          
          int c = 0;
          //while loop to make car go straight if the current heading is within +/- 5degrees of desired or taken 5 calculations
          while (newDirection > currDirection + 5 || newDirection < currDirection - 5 || c < 5)
          {
            if (turnangle >= 180)
              turnangle -= 360;
            if (turnangle <= -180)
              turnangle += 360;
            if (turnangle > maxTurnAngle)
              turnangle = maxTurnAngle;//if the necessary turn angle is greater than the max turn angle of wheels, set turn angle to max turn angle
            if (turnangle < -maxTurnAngle)
              turnangle = -maxTurnAngle;
                       
            compass.read();           
            flat = gps.location.lat();
            flon = gps.location.lng();
            newDirection = desiredHeading(flat, flon);
            if(checkwp(flat, flon))
                newDirection = desiredHeading(flat, flon);
            currDirection = compass.heading();  //find current heading from compass
            turnangle = newDirection-currDirection; //error in heading between desired and current
            c++;
           }
        }
      }     
}
*/
/* ====================================================================================================== */
//calculates desired heading using TinyGPS library functions
//returns a float heading based on the cours_to() function
float desiredHeading(float flat, float flon)
{
  float templat = lati.peek();
  float templon = longi.peek();
  
  float dH = gps.course_to(flat, flon, templat, templon);
  //account for declination angle
  dH += declinationAngle1;
  //Correct for when signs are reversed
  if (dH < 0)
    dH += 360;
  //Check for wrap due to addition of declination
  if (dH > 360)
    dH -= 360;
  return dH;      
}

/* ====================================================================================================== */
//checks to see if we are at a waypoint
//returns true if at a waypoint and pops the lati and longi stacks. false else
bool checkwp(float flat, float flon)
{
  bool la = false, lo = false;
  if(flat == lati.peek()) //maybe variance
    la = true;
  if(flon == longi.peek()) //variance
      lo = true;
  if(la && lo)
  {
    return true;
    if(!lati.isEmpty() && longi.isEmpty())
    {
      lati.pop();
      longi.pop();
    }
  }
  return false;
}

/* ====================================================================================================== */
//creates database of waypoints -- preprogrammed data
static void createDatabase()
{
  //push on the coordinates of the house for final return
  //longi.push(homelon);
  //lati.push(homelat);
  //start to end 
  long londata[] = {-121.9382074769469, -121.9382541315717, -121.9383136860484, -121.9382927427925, -121.9383501915424, -121.9383222740931, -121.9382074769469};
  long latdata[] = {37.34898548197158, 37.34906998317596, 37.34903334915039, 37.34899349300763, 37.34894896744067, 37.34891501473483, 37.34898548197158};
  int s = 7;
  
  for(int i = s-1; i >= 0; i--){
    longi.push(londata[i]);
    lati.push(latdata[i]);
  }
}

/* ====================================================================================================== */
//follows lines on the ground using 8 photoresistors on the front of the car
//returns true if not all the photo resistors are triggered
//returns false if they are.
//used in conjunction with a while loop to have continuous line following. 
//math in the function provided by Danny Mendoza's mechatronics final project car. 
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
        leftRotate();
        delay(9);
        delay(1);
    }
    
    else if(error < -1*error_diff){
        right ();
        delay(9);
        delay(1);
    }
    
    else{
        forward();
        delay(9);
        delay(1);
    }
    return true;
    
}
/* ====================================================================================================== */
//drive functions for lineFollow();
void forward()
 {
    if(Speed > lowSpeed)
      decelerate(Speed, lowSpeed);
    teamTKE.write(pos);
 } 
  
 //left
void leftRotate(){
    Serial.print("left\n");
    teamTKE.write(pos + 10);
}

//right
void right(){
    Serial.print("right\n");
    teamTKE.write(pos - 10);
}


#include <TinyGPS.h>
#include <LSM303.h>

TinyGPS gps;
LSM303 compass;

 
void setup() {
  Serial.begin(115200);
  compass.init();
  compass.enableDefault();
  Serial.println("---------------------START----------------------------------------------------");
  Serial3.print("$PMTK313,1*2E\r\n");  // Enable to search a SBAS satellite
  Serial1.print("$PMTK301,2*2E\r\n");  // Enable WAAS as DGPS Source
  Serial1.print("$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"); //causes GPS only to give out GPGGA information
  Serial1.print("$PMTK220,200*2C\r\n"); //increases rate of GPS from 1Hz to 5Hz
}
void loop(){
  float flat, flon;
  unsigned long age;
  compass.read();
  // Serial.println("compass read");
  // unsigned char cc = Serial3.read(); //read from gps
  if (gps.encode(Serial3.read())) 
  {
     gps.f_get_position(&flat, &flon, &age);
  
  
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("Lat: ");
    Serial.print(flat);
    Serial.print(", Long: ");
    Serial.println(flon);
  }
   
}

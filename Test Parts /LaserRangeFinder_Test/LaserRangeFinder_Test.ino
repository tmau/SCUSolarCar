/*
  
  Laser Range Finder Module: Basic Demonstration       
                                                         
  Author: Joe Grand [www.grandideastudio.com]             
  Contact: support@parallax.com                            
  
  Program Description:
  
  This program provides a simple demonstration of the Laser Range Finder
  Module. The distance to the target object is displayed in the Arduino
  Serial Monitor. 
  
  Please refer to the product manual for full details of system functionality 
  and capabilities.

  Revisions:
  
  1.0 (December 11, 2013): Initial release
  1.1 (April 29, 2014): Changed rxPin/txPin to use pins 10/11, respectively, for widest support across the Arduino family (http://arduino.cc/en/Reference/SoftwareSerial)
  
*/

// include the SoftwareSerial library so we can use it to talk to the LRF
#include <SoftwareSerial.h>

#define rxPin    10  // Serial input (connects to the LRF's SOUT pin)
#define txPin    11  // Serial output (connects to the LRF's SIN pin)
#define ledPin   13  // Most Arduino boards have an on-board LED on this pin

#define BUFSIZE  16  // Size of buffer (in bytes) for incoming data from the LRF (this should be adjusted to be larger than the expected response)

// set up a new serial port
SoftwareSerial lrfSerial =  SoftwareSerial(rxPin, txPin);

void setup()  // Set up code called once on start-up
{
  // define pin modes
  pinMode(ledPin, OUTPUT);
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);

  digitalWrite(ledPin, LOW);  // turn LED off
  
  // setup Arduino Serial Monitor
  Serial.begin(9600);
  while (!Serial);   // Wait until ready
  Serial.println("\n\nParallax Laser Range Finder");
  
  // set the baud rate for the SoftwareSerial port
  lrfSerial.begin(9600);

  /*
    When the LRF powers on, it launches an auto-baud routine to determine the
    host's baud rate. it will stay in this state until a "U" ($55) character
    is sent by the host. 
  */
  Serial.print("Waiting for the LRF...");
  delay(2000);                        // Delay to let LRF module start up
  lrfSerial.print('U');               // Send character
  while (lrfSerial.read() != ':');    // When the LRF has initialized and is ready, it will send a single ':' character, so wait here until we receive it
  delay(10);                          // Short delay
  lrfSerial.flush();                  // Flush the receive buffer
  Serial.println("Ready!");
  Serial.flush();                     // Wait for all bytes to be transmitted to the Serial Monitor
}

void loop()  // Main code, to run repeatedly
{
  /* 
    When a single range (R) command is sent, the LRF returns the distance to the target
    object in ASCII in millimeters. For example:
     
    D = 0123 mm
  */   
  lrfSerial.print('R');         // Send command
  digitalWrite(ledPin, HIGH);   // Turn LED on while LRF is taking a measurement
  
  // Get response back from LRF
  // See Arduino readBytesUntil() as an alternative solution to read data from the LRF
  char lrfData[BUFSIZE];  // Buffer for incoming data
  char offset = 0;        // Offset into buffer
  lrfData[0] = 0;         // Clear the buffer    
  while(1)
  {
    if (lrfSerial.available() > 0) // If there are any bytes available to read, then the LRF must have responded
    {
      lrfData[offset] = lrfSerial.read();  // Get the byte and store it in our buffer
      if (lrfData[offset] == ':')          // If a ":" character is received, all data has been sent and the LRF is ready to accept the next command
      {
        lrfData[offset] = 0; // Null terminate the string of bytes we just received
        break;               // Break out of the loop
      }
          
      offset++;  // Increment offset into array
      if (offset >= BUFSIZE) offset = 0; // If the incoming data string is longer than our buffer, wrap around to avoid going out-of-bounds
    }
  }
  Serial.println(lrfData);    // The lrfData string should now contain the data returned by the LRF, so display it on the Serial Monitor
  Serial.flush();             // Wait for all bytes to be transmitted to the Serial Monitor
 
  digitalWrite(ledPin, LOW);  // Turn LED off
}




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

  Brake(); 
  delay(2000); 

  Left();
  Forward(); 
  delay(1750); 

  Brake(); 
  delay(2000); 
  
  //Part 2
  Forward(); 
  delay(3000); 
  
  Brake(); 
  delay(2000); 

  Left(); 
  Forward();
  delay(1750); 

  Brake(); 
  delay(2000); 

//  //Part 3
//  Forward(); 
//  delay(4000); 
//
//  Brake(); 
//  delay(2000); 
//
//  Left(); 
//  delay(1000); 
//
//  Brake(); 
//  delay(2000);
  

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

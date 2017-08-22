// connect motor controller pins to Arduino digital pins
// motor one
int enA = 10;
int in1 = 9;
int in2 = 8;
int in3 = 7;
int in4 = 6;
int enB = 5;
int Speed = 200; 
int SpeedB = 255; 

void Forward();
void Reverse(); 

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
  Left(); 
  delay(2000);
  Right(); 
  delay(2000);
  Forward(); 
  delay(2000);
  Backward();
  delay(2000);
}
void Left(){
  //Left
  analogWrite(enB, SpeedB);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);  
}
void Right(){
  //Right
  analogWrite(enB, SpeedB);
  digitalWrite(in4, LOW);  
  digitalWrite(in3, HIGH);
}
void Forward()
{
  analogWrite(enA, Speed);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}
void Backward()
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

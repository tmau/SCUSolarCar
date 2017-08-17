//------------------H Bridge------------------
//Motor Control Functions 
void Forward()
{
  digitalWrite(Wheel1B, LOW);
  digitalWrite(Wheel2B, LOW);
  digitalWrite(Wheel1F, HIGH);
  digitalWrite(Wheel2F, HIGH);
  analogWrite(EnA, Speed);
  analogWrite(EnB, Speed);
}

void Forward2()
{
  digitalWrite(Wheel1B, LOW);
  digitalWrite(Wheel2B, LOW);
  digitalWrite(Wheel1F, HIGH);
  digitalWrite(Wheel2F, HIGH);
  analogWrite(EnA, Speed1);
  analogWrite(EnB, Speed2);
}

void Reverse()
{
  digitalWrite(EnA, LOW);
  digitalWrite(EnB, LOW);
  digitalWrite(Wheel1F, LOW);
  digitalWrite(Wheel2F, LOW); 
  digitalWrite(Wheel1B, HIGH);
  digitalWrite(Wheel2B, HIGH);
  analogWrite(EnA, Speed);
  analogWrite(EnB, Speed);
}

void Brake()
{
  digitalWrite(Wheel1F, LOW);
  digitalWrite(Wheel2F, LOW);
  digitalWrite(Wheel1B, LOW);
  digitalWrite(Wheel2B, LOW);
  analogWrite(EnA, Speed);
  analogWrite(EnB, Speed);
}

void Coast()
{

  digitalWrite(EnA, LOW);
  digitalWrite(EnB, LOW);
}

void PivotRight()
{
  //digitalWrite(En1, LOW);
  //digitalWrite(En2, LOW);
  digitalWrite(Wheel1B, LOW);
  digitalWrite(Wheel1F, LOW);
  digitalWrite(Wheel2B, LOW);
  digitalWrite(Wheel2F, HIGH);
  analogWrite(EnB, Speed);
}

void PivotLeft()
{
  //digitalWrite(En1, LOW);
  //digitalWrite(En2, LOW);
  digitalWrite(Wheel1B, LOW);
  digitalWrite(Wheel2F, LOW);
  digitalWrite(Wheel2B, LOW);
  digitalWrite(Wheel1F, HIGH);
  analogWrite(EnA, Speed);
}

void TurnLeft()
{
  digitalWrite(EnA, LOW);
  digitalWrite(EnB, LOW);
  digitalWrite(Wheel1B, LOW);
  digitalWrite(Wheel2F, LOW);
  digitalWrite(Wheel2B, HIGH);
  digitalWrite(Wheel1F, HIGH);
  analogWrite(EnA, Speed);
  analogWrite(EnB, Speed);
}

void TurnLeft2()
{
  digitalWrite(EnA, LOW);
  digitalWrite(EnB, LOW);
  digitalWrite(Wheel1B, LOW);
  digitalWrite(Wheel2F, LOW);
  digitalWrite(Wheel2B, HIGH);
  digitalWrite(Wheel1F, HIGH);
  analogWrite(EnA, Speed1);
  analogWrite(EnB, Speed2);
}

void TurnRight()
{
  digitalWrite(EnA, LOW);
  digitalWrite(EnB, LOW);
  digitalWrite(Wheel2B, LOW);
  digitalWrite(Wheel1F, LOW);
  digitalWrite(Wheel1B, HIGH);
  digitalWrite(Wheel2F, HIGH);
  analogWrite(EnA, Speed);
  analogWrite(EnB, Speed);
}

void TurnRight2()
{
  digitalWrite(EnA, LOW);
  digitalWrite(EnB, LOW);
  digitalWrite(Wheel2B, LOW);
  digitalWrite(Wheel1F, LOW);
  digitalWrite(Wheel1B, HIGH);
  digitalWrite(Wheel2F, HIGH);
  analogWrite(EnA, Speed1);
  analogWrite(EnB, Speed2);
}


void cmForward(int x)
{
   counter = 0;
   dist = ((360*x) /(60*PI))*10;//CHANGE
   Forward();
   while(counter < dist)
   {
    Serial.println(counter);
   }
   Brake();
}

void cmReverse(int x)
{
   counter = 0;
   dist = ((360*x) /(60*PI))*10; //CHANGE
   Reverse(); 
   while(counter > -dist)
   {
    Serial.println(counter);
   }
   Brake();
}

void KTurn()
{
  PivotLeft();
  delay(700);
  Brake();
  delay(500);
  TurnLeftReverse();
  delay(900);
  Brake();
  delay(500);
  Forward();
  delay(800);
  Brake();
  delay(500);
}

void TurnLeftReverse()
{
  //digitalWrite(En1, LOW);
  //digitalWrite(En2, LOW);
  digitalWrite(Wheel1B, LOW);
  digitalWrite(Wheel2F, LOW);
  digitalWrite(Wheel2B, HIGH);
  digitalWrite(Wheel1F, LOW);
  analogWrite(EnA, Speed);
}

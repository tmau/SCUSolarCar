//int EnA = 7;
//int EnB = 8;
//
//void setup() {
//  pinMode(EnA, INPUT);
//  pinMode(EnB, INPUT);
//  Serial.begin(9600); 
//}
//
//void loop() {
//  Serial.print(digitalRead(EnA));
//  Serial.println(digitalRead(EnB));
//}

volatile int counter;

void setup() {
  // put your setup code here, to run once:
  pinMode(18, INPUT);
  pinMode(19, INPUT);
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(18),isr,RISING); //interrupt only when rising edge is seen in CHA. 
  attachInterrupt(digitalPinToInterrupt(19),isr,RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(counter);
}

void isr()
{
  if (digitalRead(18) == 1)
  {
  counter++;
  }
  else
  {
  counter--;
  }
}


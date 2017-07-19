int EnA = 2;
int EnB = 3;

void setup() {
  pinMode(EnA, INPUT);
  pinMode(EnB, INPUT);
  Serial.begin(9600); 
}

void loop() {
  Serial.println(digitalRead(EnA));
  Serial.println(digitalRead(EnB));
}

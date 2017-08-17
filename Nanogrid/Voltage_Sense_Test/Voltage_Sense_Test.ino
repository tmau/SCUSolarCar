void setup() {
  pinMode(A2, INPUT);
  Serial.begin(9600);
}

void loop() {
//  double res = analogRead(A2)*0.1536919479/5;
 float res = analogRead(A2)*0.02797202797; //multiply by 24/858
  Serial.println(res);
}


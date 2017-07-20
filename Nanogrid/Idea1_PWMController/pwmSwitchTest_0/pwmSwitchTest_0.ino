#define in1 7
#define potent 3

int potentRead;
int pwmControl;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(in1,OUTPUT);
  //pinMode(potent,INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  potentRead=analogRead(potent);
  pwmControl=potentRead/4;
  analogWrite(in1,pwmControl);
  Serial.println(pwmControl );
}
 

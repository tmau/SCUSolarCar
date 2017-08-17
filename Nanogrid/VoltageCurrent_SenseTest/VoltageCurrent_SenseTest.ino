void setup() {
Serial.begin(9600);
pinMode(A1, INPUT);
pinMode(A2, INPUT);
//pinMode(2, OUTPUT);
//pinMode(3, OUTPUT);
}

void loop() {
int reading1 = analogRead(A1);

int reading2 = analogRead(A2);
//
Serial.print("Voltage:");
Serial.print(reading1);
Serial.print("    ");
Serial.print("Current:");
Serial.println(reading2);
//Serial.print(255);
//Serial.print(',');
//Serial.println(-255);

delay(20 );

}


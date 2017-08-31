void setup() {
  pinMode(A2, INPUT);
  Serial.begin(9600);
}

void loop() {
//  double res = analogRead(A2)*0.1536919479/5;
 float res1 = analogRead(A2)*0.02479338843; //multiply by 24/968
 float res2 = analogRead(A3)*0.02479338843;
 float res3 = analogRead(A4)*0.02479338843;
 float res4 = analogRead(A5)*0.02479338843;
 
//  Serial.println(res);
  Serial.print(res1, DEC);             // send the first value   
//  Serial.print(" , ");                   // separated by a comma
//  Serial.print(res2, DEC);             // other values get sent as above  
//  Serial.print(" , "); 
//  Serial.print(res3, DEC);  
//  Serial.print(" , "); 
//  Serial.print(res4, DEC);     
  Serial.println();                    // print a linefeed character     
  delay(1000);
}

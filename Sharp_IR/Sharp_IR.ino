// Sharp IR GP2Y0A41SK0F Distance Test
//https://electronoobs.com/eng_arduino_tut72.php

#define sensor A1 // Sharp IR GP2Y0A41SK0F (4-30cm, analog)
float filterd_IR = 0.0;
void setup() {
  Serial.begin(9600); // start the serial port
}

void loop() {

  //  float volts = analogRead(sensor);  // value from sensor * (5/1024)
  //  float calculated = (6762/(volts-9))-4;



  // 5v
  float volts = analogRead(sensor) * 0.0048828125; // value from sensor * (5/1024)
  int calculated = 13 * pow(volts, -1); // worked out from datasheet graph
  //  delay(1000); // slow down serial port
  calculated = (calculated < 4.0) ? 4.0 : (calculated > 30.0) ? 30.0 : calculated;
  filterd_IR = 0.99 * filterd_IR + 0.01 * calculated;


  //  if (distance <= 30){
  //    Serial.println(distance);   // print the distance
  //  }

  Serial.println(filterd_IR);

}

// Sharp IR GP2Y0A41SK0F Distance Test
//https://electronoobs.com/eng_arduino_tut72.php

//https://www.robot-italy.com/en/3522-sharp-distance-sensor-2y0a02-20-150cm.html


#define sensor A1 // Sharp IR GP2Y0A41SK0F (4-30cm, analog)
float filterd_IR = 0.0;
void setup() {
  Serial.begin(9600); // start the serial port
}

void loop() {

    float sensorValue = analogRead(sensor);  // value from sensor * (5/1024)
//    float calculated = (6762/(volts-9))-4;
  float calculated = 9462/(sensorValue - 16.92);
  calculated = (calculated < 20.0) ? 20.0 : (calculated > 150.0) ? 150.0 : calculated;
  filterd_IR = 0.99 * filterd_IR + 0.01 * calculated;


  //  if (distance <= 30){
  //    Serial.println(distance);   // print the distance
  //  }

  Serial.println(filterd_IR);

}

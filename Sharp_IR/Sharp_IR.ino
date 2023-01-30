// Sharp IR GP2Y0A41SK0F Distance Test
// http://tinkcore.com/sharp-ir-gp2y0a41-skf/

#define sensor A0 // Sharp IR GP2Y0A41SK0F (4-30cm, analog)

void setup() {
  Serial.begin(9600); // start the serial port
}

void loop() {
  
  float volts = analogRead(sensor);  // value from sensor * (5/1024)
  float calculated = (6762/(volts-9))-4;
  calculated = (calculated < 10.0)?10.0: (calculated > 80.0) ? 80.0:calculated; 
  Serial.println(calculated);
}

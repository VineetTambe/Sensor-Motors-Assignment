#define LIGHT_SENSOR_PIN A3 //Ambient light sensor reading 

void setup() {
  pinMode(LIGHT_SENSOR_PIN,  INPUT);
  Serial.begin(9600);
}

void loop() {
  float reading = analogRead(LIGHT_SENSOR_PIN); //Read light level
  //  bool reading = analogReads(LIGHT_SENSOR_PIN);
  if (reading > 12.0) {
    Serial.println("ON");                    //Display reading in serial monitor
  } else {
    Serial.println("OFF");
  }
}

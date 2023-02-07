#define LIGHT_SENSOR_PIN A2 //Ambient light sensor reading 
float ambient_light_ambient_light_filter = 0.0;
void setup() {
  pinMode(LIGHT_SENSOR_PIN,  INPUT);
  Serial.begin(9600);
}

void loop() {
  float reading = analogRead(LIGHT_SENSOR_PIN); //Read light level
  ambient_light_filter = 0.9 * ambient_light_filter + 0.1 * reading;
  //  bool reading = analogReads(LIGHT_SENSOR_PIN);
  //  if (reading > 12.0) {
  //    Serial.println("ON");                    //Display reading in serial monitor
  //  } else {
  //    Serial.println("OFF");
  //  }
  Serial.println(ambient_light_filter);
}

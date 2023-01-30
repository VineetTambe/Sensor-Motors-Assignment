#define DEBUG false

#ifdef DEBUG
#define BUFF_SIZE 5
#else
#define BUFF_SIZE 4
#endif

#define POTENTIOMETER_PIN A0

uint8_t arr[BUFF_SIZE];
bool hasRead = false;
void setup() {
  pinMode(POTENTIOMETER_PIN, INPUT);
  Serial.begin(9600);
}

void loop() {
  int pot_data = analogRead(POTENTIOMETER_PIN);
  uint8_t percentage = map(pot_data, 0, 1023, 0, 255);
//  if (Serial.available() >= (BUFF_SIZE)) {
//    hasRead = false;
//    int temp = Serial.readBytes(arr, BUFF_SIZE);
//    hasRead = true;
//  }
//  if (hasRead) {
//    Serial.print("\nSuccessfully read the following");
//    Serial.print("\t");
//    for (uint8_t i = 0 ; i < BUFF_SIZE; i++) {
//      Serial.write(arr[i]);
//      Serial.print("\t");
//    }
//    Serial.println();
//    hasRead = false;
//    Serial.flush();
//  }
  
  Serial.write(percentage);
  delay(50);
//  Serial.println(percentage);
}

#define DEBUG false

#ifdef DEBUG
#define BUFF_SIZE 5
#else
#define BUFF_SIZE 4
#endif

uint8_t arr[BUFF_SIZE];
bool hasRead = false;
void setup() {
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() >= (BUFF_SIZE)) {
    hasRead = false;
    int temp = Serial.readBytes(arr, BUFF_SIZE);
    hasRead = true;
  }
  
  if (hasRead) {
    Serial.print("\nSuccessfully read the following");
    Serial.print("\t");
    for (uint8_t i = 0 ; i < BUFF_SIZE; i++) {
      Serial.write(arr[i]);
      Serial.print("\t");
    }
    Serial.println();
    hasRead = false;
    Serial.flush();
  }
}

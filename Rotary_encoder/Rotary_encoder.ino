#define ENCODER_PIN 2
volatile long encoder_val = 0;
void encoderISR() {
  encoder_val++;
}

void setup()
{
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, RISING);
  Serial.begin(9600);
}
void loop()
{
  Serial.println(encoder_val);
  delay(50);
}

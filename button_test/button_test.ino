#define CONTROL_BUTTON 3         // digital pin 3
#define BUTTON_DEBOUNCE_TIME 500  // 50 ms

volatile unsigned long buttonTimer = 0 ;
bool control_mode = true;
void buttonISR() {
  /** This Interrupt Service Routine increments the button0 state value when pressed */
  if (buttonTimer == 0 || (millis() - buttonTimer) > BUTTON_DEBOUNCE_TIME) {
    buttonTimer = millis();
    control_mode = !control_mode;

  }
}
void setup() {
  // put your setup code here, to run once:
  attachInterrupt(digitalPinToInterrupt(CONTROL_BUTTON), buttonISR, RISING);
  Serial.begin(9600);
}

void loop() {
  if (control_mode == true) {
    Serial.println(1);
  } else {
    Serial.println(0);
  }
  // put your main code here, to run repeatedly:

}

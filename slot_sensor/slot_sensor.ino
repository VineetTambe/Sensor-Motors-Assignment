//https://www.instructables.com/Photomicrosensor-to-Arduino/

const int PhotoIn = 2; //input pin for Photomicrosensor
const int LED = 13; //output pin for LED
int State = 0; //a variable to read the encoder state
void setup()
{
  pinMode(PhotoIn, INPUT); //set pin 2 as input
  pinMode(LED, OUTPUT); //set pin 13 as output
  Serial.begin(9600);
}
void loop() {
  State = digitalRead(PhotoIn);
  if (State == HIGH) {  //if the encoder output is in a high logical state
    digitalWrite(LED, HIGH); //turn the LED on
    Serial.println("High");
  }
  else {
    digitalWrite(LED, LOW); //turn the LED off
    Serial.println("Low");
  }
}

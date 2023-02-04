/*
  Ultrasonic Sensor HC-SR04 interfacing with Arduino.
*/
// defining the pins
#define MOVING_AVERAGE_FILTER_SIZE 15

const int trigPin = 4;
const int echoPin = 5;
uint8_t ma_counter = 0;
// defining variables
long duration;
int distance;
void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input 
  Serial.begin(9600); // Starts the serial communication
}

float moving_average_ultrasonic;
float moving_average_accumulator[MOVING_AVERAGE_FILTER_SIZE];

void loop() {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  distance = min (100, distance);
  moving_average_ultrasonic = ((0.9) * moving_average_ultrasonic + 0.1 * distance;
  
  Serial.print("Distance: ");
  Serial.println(moving_average_ultrasonic);




  
}

//void loop() {
//  // Clears the trigPin
//  digitalWrite(trigPin, LOW);
//  delayMicroseconds(2);
//  // Sets the trigPin on HIGH state for 10 micro seconds
//  digitalWrite(trigPin, HIGH);
//  delayMicroseconds(10);
//  digitalWrite(trigPin, LOW);
//  // Reads the echoPin, returns the sound wave travel time in microseconds
//  duration = pulseIn(echoPin, HIGH);
//  // Calculating the distance
//  distance = duration * 0.034 / 2;
//  // Prints the distance on the Serial Monitor
//  Serial.print("Distance: ");
//  Serial.println(min(100.0,distance));
//}

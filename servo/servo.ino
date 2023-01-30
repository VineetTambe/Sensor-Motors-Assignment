#include <Servo.h> 
// Declare the Servo pin 
#define servoPin 3 
// Create a servo object 
Servo Servo1; 
void setup() { 
   // We need to attach the servo to the used pin number 
   Servo1.attach(servoPin); 
   Serial.begin(9600);
}
void loop(){ 

  if (Serial.available() > 1){
    int val = Serial.parseInt();
    
    if (val != 0){


      val = min (180, max(val,4));
      Servo1.write(val); 
      }
    
    


    Serial.println(val);
  }
   // Make servo go to 0 degrees 
//   
//   delay(1000); 
//   // Make servo go to 90 degrees 
//   Servo1.write(90); 
//   delay(1000); 
//   // Make servo go to 180 degrees 
//   Servo1.write(180); 
//   delay(1000); 
}

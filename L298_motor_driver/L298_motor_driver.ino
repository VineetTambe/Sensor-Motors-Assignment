#include "RotaryEncoder.h"

#define dirPin1 8
#define dirPin2 7
#define motorEnPin 5

#define LOOP_DT 500 // micro seconds


int val = 0;
RotaryEncoder encoder(A0, A2, 5, 6, 3000);


void actuate_motor( int dir , int pwm , int dir_pin1, int dir_pin2, int en_pin) {

  switch (dir) {
    case -1:
      digitalWrite(dir_pin1, HIGH);
      digitalWrite(dir_pin2, LOW);
      analogWrite(en_pin, pwm);
//      Serial.println("moving backwards");
      break;
    case 0:
      digitalWrite(dir_pin1, LOW);
      digitalWrite(dir_pin2, LOW);
      analogWrite(en_pin, 0);
      break;
    case 1:
      digitalWrite(dir_pin1, LOW);
      digitalWrite(dir_pin2, HIGH);
      analogWrite(en_pin, pwm);
//      Serial.println("moving forwards");
      break;
  }
}


void setup() {
  // put your setup code here, to run once:
  pinMode(dirPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(motorEnPin, OUTPUT);
  actuate_motor(0, 0, dirPin1, dirPin2, motorEnPin);
  Serial.begin(9600);
}


int target = 0;
int dir = 0;
int prev_error = 0;
int error = 0 ;
int d_error = 0;
int _stop = 1;

float kp = 7.5;
float kd = 0;
float ki = 0;
int pwm_out = 0;
int errSum = 0;
void loop() {
  // put your main code here, to run repeatedly:

  if (Serial.available() >= 2) {
    _stop = Serial.parseInt();
    target = Serial.parseInt();
    val = 0;
    errSum = 0;
    Serial.print("\nSetting dir = ");
    Serial.println(dir);

    Serial.print("\nSetting Tartget = ");
    Serial.println(target);
  }

  int enc = encoder.readEncoder();
  if (enc != 0) {
    val = val + (enc);
    Serial.print("\nEncoder Value = ");
    Serial.println(val);
  }
  return;

  // simple pid calculations 
  
  error = target - val;

  if (_stop && abs(error) > 5) {
    
    d_error = (prev_error - error)/LOOP_DT;
    errSum += error * LOOP_DT;
    pwm_out = (int) kp * error + kd * d_error + ki * errSum;

    dir = (error <= 0) ? 1 : -1;

    pwm_out = abs(pwm_out);
    pwm_out = max(0, min(pwm_out, 255));

    prev_error = error;

    actuate_motor(dir, pwm_out, dirPin1, dirPin2, motorEnPin);
  } else {
    actuate_motor(_stop, pwm_out, dirPin1, dirPin2, motorEnPin);
  }

  Serial.print("\n Error = \t");
  Serial.print(error);

  Serial.print(" PWM = \t");
  Serial.print(pwm_out);
//
//
//  Serial.print("  Setting dir = \t ");
//  Serial.print(dir);
//
//  Serial.print("  Setting Tartget = \t");
//  Serial.println(target);
  
  delayMicroseconds(LOOP_DT);
}

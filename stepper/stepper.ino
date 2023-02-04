/*Example sketch to control a stepper motor with DRV8825 stepper motor driver, AccelStepper library and Arduino: number of steps or revolutions. More info: https://www.makerguides.com */
//https://lastminuteengineers.com/drv8825-stepper-motor-driver-arduino-tutorial/

//https://www.makerguides.com/drv8825-stepper-motor-driver-arduino-tutorial/
// Include the AccelStepper library:
#include <AccelStepper.h>

// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
#define STEPPER_DIR_PIN 12 // 2
#define STEPPER_STEP_PIN 11 // 3
#define MOTOR_INTERFACE_TYPE 1
#define DEFAULT_STEPPER_SPEED 400

// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(MOTOR_INTERFACE_TYPE, STEPPER_STEP_PIN, STEPPER_DIR_PIN);
bool start = false;
void setup() {
  // Set the maximum speed in steps per second:
  stepper.setMaxSpeed(1000);
  Serial.begin(9600);
}
int angle = 0;
void moveStepper(int dir, int angle) {
//  int num_steps = (int) (250.0 * angle / 255.0);
  int num_steps = (int) (400.0 * angle/ 360.0 );
  //  int stepper_speed = ((num_steps > 0) ? 1 : -1) * DEFAULT_STEPPER_SPEED;\
  // Reset the position to 0:
  stepper.setCurrentPosition(0);
  Serial.println(dir);
  while (stepper.currentPosition() !=  dir * num_steps)
  {
    stepper.setSpeed(dir * DEFAULT_STEPPER_SPEED);
    stepper.runSpeed();
  }
}

void loop() {

  if (Serial.available() > 1) {
    angle = Serial.parseInt();
    if (angle == 0) {
      start = false;
    } else {
      start = true;
    }
  }

  if (!start) return;

  Serial.print("starting with angle: ");
  Serial.println(angle);

  // Reset the position to 0:
  stepper.setCurrentPosition(0);


  //  angle = angle * 360 / 255.0
  //
  //  int num_steps = (int) (400.0 * angle * 360.0 / 255.0);


  //  int num_steps = (int) (400.0 * angle / 360.0);
  //
  //  int stepper_speed = ((num_steps > 0) ? 1 : -1) * DEFAULT_STEPPER_SPEED;
  //  while (stepper.currentPosition() != num_steps)
  //  {
  //    stepper.setSpeed(stepper_speed);
  //    stepper.runSpeed();
  //  }

  moveStepper((angle > 0) ? 1 : -1, abs(angle));


  delay(1000);
  start = false;
}

//void loop() {
//
//  if (Serial.available() > 1) {
//    angle = Serial.parseInt();
//    if (angle == 0){
//      start = false;
//    } else {
//      start = true;
//    }
//  }
//
//  if (!start) return;
//
//  Serial.print("starting with angle: ");
//  Serial.println(angle);
//
//  // Reset the position to 0:
////  stepper.setCurrentPosition(0);
//
//  // Run the motor forward at 400 steps/second until the motor reaches 1200 steps (3 revolutions):
//  int num_steps = (int) (1200.0 * angle / 360.0);
//  int stepper_speed = ((num_steps > 0)? 1 : -1) * DEFAULT_STEPPER_SPEED;
//
//  Serial.print("num_steps = ");
//  Serial.println(num_steps);
//
//  Serial.print("calculated stepper_speed = ");
//  Serial.println(stepper_speed);
////  while(stepper.currentPosition() != num_steps)
////  {
////    stepper.setSpeed(stepper_speed);
////    stepper.runSpeed();
////  }
//  delay(1000);
//  start = false ;
//}

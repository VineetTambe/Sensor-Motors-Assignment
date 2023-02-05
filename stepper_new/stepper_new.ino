// ProportionalControl.pde
// -*- mode: C++ -*-
//
// Make a single stepper follow the analog value read from a pot or whatever
// The stepper will move at a constant speed to each newly set posiiton,
// depending on the value of the pot.
//
// Copyright (C) 2012 Mike McCauley
// $Id: ProportionalControl.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $

#include <AccelStepper.h>

#define STEPPER_DIR_PIN 12 // 2
#define STEPPER_STEP_PIN 11 // 3
#define MOTOR_INTERFACE_TYPE 1
#define DEFAULT_STEPPER_SPEED 400

// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(MOTOR_INTERFACE_TYPE, STEPPER_STEP_PIN, STEPPER_DIR_PIN);

// This defines the analog input pin for reading the control voltage
// Tested with a 10k linear pot between 5v and GND
#define ANALOG_IN A0

void setup()
{
  stepper.setMaxSpeed(1000);
  Serial.begin(9600);

}
int angle = 0;
bool start;
void loop()
{
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


  stepper.moveTo(angle);
  stepper.setSpeed(angle);
  stepper.runSpeedToPosition();
}

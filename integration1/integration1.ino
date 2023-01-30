#include <Servo.h> // disables pwm on pin 10 and 9
#include <AccelStepper.h>

#define DEBUG false
#define POTENTIOMETER_PIN A0
#define SHARP_IR_PIN A1 // Sharp IR GP2Y0A41SK0F (10-80cm, analog)
#define ULTRASONIC_TRIGGER_PIN 4
#define ULTRASONIC_ECHO_PIN 5
#define SERVO_PIN 3
#define SLOT_SENSOR_PIN 2
#define STEPPER_DIR_PIN 12 // 2
#define STEPPER_STEP_PIN 11 // 3
#define MOTOR_INTERFACE_TYPE 1
#define DEFAULT_STEPPER_SPEED 400

#if DEBUG
#define READ_BUFF_SIZE 8
#define WRITE_BUFF_SIZE 5
#else
#define READ_BUFF_SIZE 7
#define WRITE_BUFF_SIZE 4
#endif

/**
   Write buffer is a byte array of size four. With the following order of elements:
   1. Potentiometer reading with mapping (0-255) to (0-5V)
   2. SharpIR sensor reading with mapping (0-255) to (10cm-80cm)
   3. Ultrasonic sensor
   4. Slot Sensor 0 or 255 to indicate ON or OFF

   Read Buffer  is a byte buffer of size seven used to control the actuators:
   0. which actuator to control: 0 - STOP & RESET everything, 1 - stepper, 2 - servo, 3 - dc motor-position, 4 - dc motor-velocity
   1. stepper
   2. servo
   3. dc-motor - position control - direction
   4. dc-motor - position control - target position (degrees 0 to 360 0 to -360)
   5. dc-motor - velocity control - direction
   6. dc-motor - velocity control - target velocity (rpm)

   To just stop the DC motor - send 3 or 4 and target position/velocity = 0
*/

uint8_t read_buff[READ_BUFF_SIZE];
uint8_t write_buff[WRITE_BUFF_SIZE];
uint8_t control_var[1];
uint8_t ultrasonic_distance;
bool hasRead = false;
bool hasWritten = false;

// Create a servo object
Servo servo;

// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(MOTOR_INTERFACE_TYPE, STEPPER_STEP_PIN, STEPPER_DIR_PIN);

uint8_t updateUltrasonicData() {
  // Clears the trigPin
  digitalWrite(ULTRASONIC_TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(ULTRASONIC_TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIGGER_PIN, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);
  // Calculating the distance in cm
  return (uint8_t) min(255, (duration * 0.034 / 2));
}

void populateWriteBuff(uint8_t (&wb)[WRITE_BUFF_SIZE]) {
  int pot_data = analogRead(POTENTIOMETER_PIN);
  float ir_data = (6762 / (analogRead(SHARP_IR_PIN) - 9)) - 4; // value from sensor * (5/1024)
  ir_data = (uint8_t )((ir_data < 10.0) ? 10.0 : (ir_data > 80.0) ? 80.0 : ir_data) * 255 / 80;
  wb[0] = map(pot_data, 0, 1023, 0, 255);
  wb[1] = ir_data;
  wb[2] = ultrasonic_distance;
  wb[3] = (digitalRead(SLOT_SENSOR_PIN) == HIGH) ? 255 : 0;
}

void stopAndResetAllActuators() {
  // Reset the position to 0:
  stepper.setCurrentPosition(0);

  // reset servo to 0deg
  servo.write(4);

  // stop DC motor

}

void moveStepper(int angle) {
  int num_steps = (int) (400.0 * angle * 360.0 / 255.0);
  int stepper_speed = ((num_steps > 0) ? 1 : -1) * DEFAULT_STEPPER_SPEED;
  while (stepper.currentPosition() != num_steps)
  {
    stepper.setSpeed(stepper_speed);
    stepper.runSpeed();
  }
}

void moveServo(int angle) {
  servo.write(min(180, max(angle, 4)));
}
void triggerActions(uint8_t rb[READ_BUFF_SIZE]) {
  switch ((int)rb[0] - 48) {
    case 0:
      stopAndResetAllActuators();
      break;
    case 1:
      moveStepper((int)rb[1]);
      break;
    case 2:
      moveServo((int)rb[2]);
      break;
    case 3:
      break;
    case 4:
      break;
  }
}

void setup() {
  pinMode(POTENTIOMETER_PIN, INPUT);
  pinMode(SHARP_IR_PIN, INPUT);
  pinMode(ULTRASONIC_TRIGGER_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  pinMode(SLOT_SENSOR_PIN, INPUT);
  servo.attach(SERVO_PIN);
  stepper.setMaxSpeed(1000);

  stopAndResetAllActuators();
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    // read for 'r'
    int temp = Serial.readBytes(control_var, 1);

    if ((char)control_var[0] == 'r') {

      populateWriteBuff(write_buff);
      for (uint8_t i = 0 ; i < WRITE_BUFF_SIZE; i++) {
        Serial.write(write_buff[i]);
      }
      // delay(50);
    } else if ((char)control_var[0] == 'w') {
      Serial.flush();
      delay(10);
      temp = Serial.readBytes(read_buff, READ_BUFF_SIZE);
      triggerActions(read_buff);
    }
  }
  ultrasonic_distance = updateUltrasonicData();
}

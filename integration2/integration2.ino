#include <Servo.h> // disables pwm on pin 10 and 9
#include <AccelStepper.h>

#define DEBUG false
#define POTENTIOMETER_PIN A0
#define SHARP_IR_PIN A1 // Sharp IR GP2Y0A41SK0F (10-80cm, analog)
#define ULTRASONIC_TRIGGER_PIN 4
#define ULTRASONIC_ECHO_PIN 6
#define SERVO_PIN 3
#define STEPPER_DIR_PIN 12 // 2
#define STEPPER_STEP_PIN 11 // 3
#define MOTOR_INTERFACE_TYPE 1
#define DEFAULT_STEPPER_SPEED 400
#define LIGHT_SENSOR_PIN A3 //Ambient light sensor reading 


// DC motor params
#define ENCODER_PIN 2
#define DIR_PIN1 8
#define DIR_PIN2 7
#define MOTOR_ENABLE_PIN 5
#define CLICKS_PER_ROTATION 98
#define LOOP_DT 50 // micro seconds

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
volatile long encoder_val = 0;

// position control errors
int target = 0;
int dir = 0;
int prev_error = 0;
int error = 0 ;
int d_error = 0;
int _stop = 1;

float kp_pos = 2;
float kd_pos = 0.00001;
float ki_pos = 0.000001;
int pwm_out = 0;
int errSum = 0;
int prev_encoder_val = 0;
bool start_dc_motor_position_control;

//velocity control
int target_rpm = 0;
int prev_error_vel = 0;
int error_vel = 0 ;
int d_error_vel = 0;

float kp_vel = 1.5;
float kd_vel = 0.000001;
float ki_vel = 0.000;
int pwm_out_vel = 0;
float errSum_vel = 0;
float motor_rpm = 0;
bool start_dc_motor_velocity_control;
// Create a servo object
Servo servo;

// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(MOTOR_INTERFACE_TYPE, STEPPER_STEP_PIN, STEPPER_DIR_PIN);

void encoderISR() {
  encoder_val++;
}

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
  wb[3] = ((int)analogRead(LIGHT_SENSOR_PIN) > 13) ? 255 : 0;
}

void actuate_motor( int dir , int pwm , int dir_pin1, int dir_pin2, int en_pin) {

  switch (dir) {
    case -1:
      digitalWrite(dir_pin1, HIGH);
      digitalWrite(dir_pin2, LOW);
      analogWrite(en_pin, pwm);
      Serial.println("moving backwards");
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
      Serial.println("moving forwards");
      break;
  }
}

void stopAndResetAllActuators() {
  // Reset the position to 0:
  stepper.setCurrentPosition(0);

  // reset servo to 0deg
  servo.write(4);

  // stop DC motor
  actuate_motor(0 , 0, DIR_PIN1, DIR_PIN2, MOTOR_ENABLE_PIN);
  start_dc_motor_position_control = false;
  start_dc_motor_velocity_control = false;
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

void clearControlVarsPos() {
  encoder_val = 0;
  prev_encoder_val = 0;
  errSum = 0;
}

void clearControlVarsVelocity() {
  encoder_val = 0;
  prev_encoder_val = 0;
  errSum_vel = 0;
  pwm_out_vel = 0;
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
      dir = (int)rb[3];
      target = (int)rb[4];
      start_dc_motor_position_control = true;
      start_dc_motor_velocity_control = false;
      clearControlVarsPos();
      clearControlVarsVelocity();
      break;
    case 4:
      dir = (int)rb[5];
      target_rpm = (int)rb[6];
      start_dc_motor_position_control = false;
      start_dc_motor_velocity_control = true;
      clearControlVarsPos();
      clearControlVarsVelocity();
      break;
  }
}

void positionPID() {
  if (start_dc_motor_position_control) {

    // simple pid calculations for position control
    error = target - encoder_val;

    if (abs(error) > 10) {
      d_error = (prev_error - error) / LOOP_DT;
      errSum += error * LOOP_DT;

      pwm_out = (int) kp_pos * error + kd_pos * d_error + ki_pos * errSum;
      pwm_out = abs(pwm_out);
      pwm_out = max(0, min(pwm_out, 255));
      prev_error = error;
      actuate_motor(dir, pwm_out, DIR_PIN1, DIR_PIN2, MOTOR_ENABLE_PIN);
    } else {
      pwm_out = 0;
      actuate_motor(0, pwm_out, DIR_PIN1, DIR_PIN2, MOTOR_ENABLE_PIN);
    }
  }

}

void velocityPID() {
  if (start_dc_motor_velocity_control) {
    motor_rpm = (encoder_val - prev_encoder_val) / CLICKS_PER_ROTATION;
    motor_rpm = motor_rpm * 1000 * 60 / LOOP_DT;

    prev_encoder_val = encoder_val;
    // simple pid calculations for velocity control
    error_vel = target_rpm - motor_rpm;
    if (abs(error_vel) > 5) {
      d_error_vel = (prev_error_vel - error_vel) / LOOP_DT;
      errSum_vel += error_vel * LOOP_DT;
      int incremental_pwm = (int) kp_vel * error_vel + kd_vel * d_error_vel + ki_vel * errSum_vel;

      pwm_out_vel += incremental_pwm;
      prev_error_vel = error_vel;
    }
    actuate_motor(dir, pwm_out_vel, DIR_PIN1, DIR_PIN2, MOTOR_ENABLE_PIN);
  }
}

void setup() {
  pinMode(POTENTIOMETER_PIN, INPUT);
  pinMode(SHARP_IR_PIN, INPUT);
  pinMode(ULTRASONIC_TRIGGER_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  pinMode(LIGHT_SENSOR_PIN,  INPUT);

  //dc motor
  pinMode(DIR_PIN1, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, RISING);

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
  positionPID();
  velocityPID();
}

#define ENCODER_PIN 2

#define DIR_PIN1 8
#define DIR_PIN2 7
#define MOTOR_ENABLE_PIN 6
#define CLICKS_PER_ROTATION 98
#define LOOP_DT 50 // micro seconds

volatile long encoder_val = 0;

int target_rpm = 0;
int dir = 0;
int prev_error_vel = 0;
int error_vel = 0 ;
int d_error_vel = 0;
int _stop = 1;

float kp_vel = 1.5;
float kd_vel = 0.000001;
float ki_vel = 0.000;
int pwm_out_vel = 0;
float errSum_vel = 0;
float prev_encoder_val = 0;
float motor_rpm = 0;

void encoderISR() {
  encoder_val++;
}

void actuate_motor( int dir , int pwm , int dir_pin1, int dir_pin2, int en_pin) {

  switch (dir) {
    case -1:
      digitalWrite(dir_pin1, HIGH);
      digitalWrite(dir_pin2, LOW);
      analogWrite(en_pin, pwm);
//            Serial.println("moving backwards");
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
//            Serial.println("moving forwards");
      break;
  }
}


void setup() {
  // put your setup code here, to run once:
  pinMode(DIR_PIN1, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, RISING);

  Serial.begin(9600);
}


void loop() {
  // put your main code here, to run repeatedly:

  if (Serial.available() >= 2) {
    _stop = Serial.parseInt();
    target_rpm = Serial.parseInt();

    encoder_val = 0;
    prev_encoder_val = 0;
    errSum_vel = 0;
    pwm_out_vel = 0;
    dir = (target_rpm <= 0) ? 1 : -1;
    target_rpm = abs(target_rpm);


    Serial.print("\nSetting stop = ");
    Serial.println(_stop);

    Serial.print("\nSetting dir = ");
    Serial.println(dir);

    Serial.print("\nSetting Tartget = ");
    Serial.println(target_rpm);

  }

  if (_stop == 0) {
    pwm_out_vel = 0;
    actuate_motor(_stop, pwm_out_vel, DIR_PIN1, DIR_PIN2, MOTOR_ENABLE_PIN);
    return;
  }
  //  Serial.println
  motor_rpm = (encoder_val - prev_encoder_val) / CLICKS_PER_ROTATION;
  motor_rpm = motor_rpm * 1000 * 60 / LOOP_DT;

  prev_encoder_val = encoder_val;
  // simple pid calculations for velocity control
  error_vel = target_rpm - motor_rpm;
  if (abs(error_vel) > 5) {
    d_error_vel = (error_vel - prev_error_vel) / LOOP_DT;
    errSum_vel += error_vel * LOOP_DT;
    int incremental_pwm = (int) kp_vel * error_vel + kd_vel * d_error_vel + ki_vel * errSum_vel;
    pwm_out_vel += incremental_pwm;
    prev_error_vel = error_vel;
  }
  
  actuate_motor(dir, pwm_out_vel, DIR_PIN1, DIR_PIN2, MOTOR_ENABLE_PIN);
  Serial.print("\n motor_rpm = \t");
  Serial.print((uint8_t) motor_rpm);
  delayMicroseconds(LOOP_DT);
}

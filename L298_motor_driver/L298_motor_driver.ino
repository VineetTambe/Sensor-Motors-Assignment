#define ENCODER_PIN 2

#define DIR_PIN1 8
#define DIR_PIN2 7
#define MOTOR_ENABLE_PIN 6
#define CLICKS_PER_ROTATION 98
#define LOOP_DT 50 // micro seconds
volatile long encoder_val = 0;

// position control errors
int target = 0;
int dir = 0;
int prev_error = 0;
int error = 0 ;
int d_error = 0;
int _stop = 1;

float kp_pos = 5.25;
float kd_pos = 0.00000001;
float ki_pos = 0.00000001;
int pwm_out = 0;
int errSum = 0;
int prev_encoder_val = 0;
bool start_dc_motor_position_control;
int feedForwards = 80;


void encoderISR() {
  encoder_val++;
}

void actuate_motor( int dir , int pwm , int dir_pin1, int dir_pin2, int en_pin) {
//  pinMode(MOTOR_ENABLE_PIN, INPUT);
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
    target = Serial.parseInt();
    
    encoder_val = 0;
    prev_encoder_val = 0;
    errSum = 0;

    dir = (target <= 0) ? 1 : -1;
    target = abs(target);

    target = (int) (target / 5.0);

    Serial.print("\nSetting dir = ");
    Serial.println(dir);

    Serial.print("\nSetting Tartget = ");
    Serial.println(target);

  }


    error = target - encoder_val;

    if (error > 5) {
      d_error = (prev_error - error) / LOOP_DT;
      errSum += error * LOOP_DT;

//      pwm_out = (int) 100 + kp_pos * error;
      pwm_out = (int) feedForwards +  kp_pos * error + kd_pos * d_error + ki_pos * errSum;
//      pwm_out = 120;
//      pwm_out = s
      pwm_out = abs(pwm_out);
      pwm_out = max(0, min(pwm_out, 255));
      prev_error = error;
      actuate_motor(dir, pwm_out, DIR_PIN1, DIR_PIN2, MOTOR_ENABLE_PIN);
    } else {
      pwm_out = 0;
      actuate_motor(0, pwm_out, DIR_PIN1, DIR_PIN2, MOTOR_ENABLE_PIN);
    }

//  // simple pid calculations for position control
//  error = target - encoder_val;
//
//  if (_stop && abs(error) > 20) {
//
//    d_error = (prev_error - error) / LOOP_DT;
//    errSum += error * LOOP_DT;
//    
//    pwm_out = (int) kp_pos * error + kd_pos * d_error + ki_pos * errSum;
//    pwm_out = abs(pwm_out);
//    pwm_out = max(0, min(pwm_out, 255));
//    prev_error = error;
//    actuate_motor(dir, pwm_out, DIR_PIN1, DIR_PIN2, MOTOR_ENABLE_PIN);
//  } else {
//    pwm_out = 0;
//    actuate_motor(_stop, pwm_out, DIR_PIN1, DIR_PIN2, MOTOR_ENABLE_PIN);
//  }


//  Serial.print("\n encoder_val = \t");
//  Serial.print(encoder_val);
//  
  Serial.print("\n Error = \t");
  Serial.print(error);
  
  Serial.print(" PWM = \t");
  Serial.print(pwm_out);
  
  delayMicroseconds(LOOP_DT);
}

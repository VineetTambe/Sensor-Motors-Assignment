#define ENCODER_PIN 2

#define DIR_PIN1 8
#define DIR_PIN2 7
#define MOTOR_ENABLE_PIN 5
#define CLICKS_PER_ROTATION 98
#define LOOP_DT 50 // micro seconds
volatile long encoder_val = 0;

void encoderISR() {
  encoder_val++;
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

    Serial.print("\nSetting dir = ");
    Serial.println(dir);

    Serial.print("\nSetting Tartget = ");
    Serial.println(target);

  }


  // simple pid calculations for position control
  error = target - encoder_val;

  if (_stop && abs(error) > 10) {

    d_error = (prev_error - error) / LOOP_DT;
    errSum += error * LOOP_DT;
    
    pwm_out = (int) kp_pos * error + kd_pos * d_error + ki_pos * errSum;
    pwm_out = abs(pwm_out);
    pwm_out = max(0, min(pwm_out, 255));
    prev_error = error;
    actuate_motor(dir, pwm_out, DIR_PIN1, DIR_PIN2, MOTOR_ENABLE_PIN);
  } else {
    pwm_out = 0;
    actuate_motor(_stop, pwm_out, DIR_PIN1, DIR_PIN2, MOTOR_ENABLE_PIN);
  }

  Serial.print("\n Error = \t");
  Serial.print(error);
  
  Serial.print(" PWM = \t");
  Serial.print(pwm_out);
  
  delayMicroseconds(LOOP_DT);
}

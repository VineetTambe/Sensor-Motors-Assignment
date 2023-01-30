#define DEBUG false

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
   1. which actuator to control: 0 - STOP & RESET everything, 1 - stepper, 2 - servo, 3 - dc motor-position, 4 - dc motor-velocity
   2. stepper
   3. servo
   4. dc-motor - position control - direction
   5. dc-motor - position control - target position (degrees 0 to 360 0 to -360)
   6. dc-motor - velocity control - direction
   7. dc-motor - velocity control - target velocity (rpm)

   To just stop the DC motor - send 3 or 4 and target position/velocity = 0
*/

uint8_t read_buff[READ_BUFF_SIZE];
uint8_t write_buff[WRITE_BUFF_SIZE];
uint8_t control_var[1];
bool hasRead = false;
bool hasWritten = false;


int led = 13;


void setup() {
  pinMode(led, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {

    // read for 'r' or 'w'
    int temp = Serial.readBytes(control_var, 1);

    if ((char)control_var[0] == 'r') {

      //      hasRead = false;
      //      temp = Serial.readBytes(arr, READ_BUFF_SIZE);
      //      hasRead = true;
      //      triggerActions();

//      digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
//      delay(200);               // wait for a second
//      digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
    } else if ((char)control_var[0] == 'w') {
      Serial.flush();
      read_buff[0] = 0;
      delay(10);
      
      int temp = Serial.readBytes(read_buff, READ_BUFF_SIZE);
      //      hasWritten = false;
      //      for (uint8_t i = 0 ; i < WRITE_BUFF_SIZE; i++) {
      //        Serial.write(write_buff[i]);
      //      }
      //      hasWritten = true;

      for (int i = 0; i < (int)read_buff[0] - 48; i++ ) {
        digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
        delay(200);               // wait for a second
        digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
        delay(200);               // wait for a second

      }
    }
  }
}

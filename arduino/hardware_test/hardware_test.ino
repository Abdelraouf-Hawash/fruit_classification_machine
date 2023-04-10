// By : abdelraouf Hawash
// in : 7/4/2023
// this code should move srevo rihgt and left, motor should move forward and backward then read encoder position and print it on serial

const int pwm_motor_pin = 10; // motor pins
const int inA_motor_pin = 8 ;  
const int inB_motor_pin = 9 ;
const int servo_pin = 6;      // servo pin (we will control servo manualy to use anther pin as PWM)

#define encoder_PinA 3   //encoder chanals
#define encoder_PinB 5

int speed_pwm = 250;           //PWM commond for the motor
int max_pwm = 255;             //maximum pwm command for motors
const int min_servo_pos = 40;  // minimum position for servo
const int max_servo_pos = 150; // maximum position for servo

volatile float encoder_pos = 0;              //encoder position counter
const double encoder_Puls_per_cm = 62.5; // encoder puls per cm


void setup() {

  //initialization serial for communication
  Serial.begin(115200);
  Serial.setTimeout(10);
  
  // motor pins mode
  pinMode (pwm_motor_pin, OUTPUT); // motor
  pinMode (inA_motor_pin, OUTPUT); 
  pinMode (inB_motor_pin, OUTPUT);
  
  //servo pins
  pinMode(servo_pin,OUTPUT);

  // encoder pins 
  pinMode (encoder_PinA, INPUT_PULLUP);
  pinMode (encoder_PinB, INPUT_PULLUP); 
  attachInterrupt (digitalPinToInterrupt(encoder_PinA), read_encoder, RISING);

}

void loop() {

  set_motor( speed_pwm,pwm_motor_pin,inA_motor_pin,inB_motor_pin);
  // speed_pwm = -speed_pwm;
  
  for (int pos = min_servo_pos; pos <= max_servo_pos; pos += 1) { // goes from min degrees to max degrees
    // in steps of 1 degree
    servo_write(pos,servo_pin);     // tell servo to go to position in variable 'pos'
    delay(1);                       // waits 1 ms for the servo to reach the position
    Serial.print(encoder_pos);       // print encoder position to serial
    Serial.print(" , ");
    Serial.println(round(encoder_pos / encoder_Puls_per_cm)); // print position in cm
  }

  for (int pos = max_servo_pos; pos >= min_servo_pos; pos -= 1) { // goes from max degrees to min degrees
    servo_write(pos,servo_pin);      // tell servo to go to position in variable 'pos'
    delay(1);                       // waits 1 ms for the servo to reach the position
    Serial.print(encoder_pos);       // print encoder position to serial
    Serial.print(" , ");
    Serial.println(round(encoder_pos / encoder_Puls_per_cm)); // print position in cm
  }
}

// setting motors pwm
void set_motor( int PWM_vel ,int pwm_pin ,int inpA ,int inpB ){

  PWM_vel = constrain(PWM_vel,-max_pwm,max_pwm);
  
  if(PWM_vel > 0 ){
    analogWrite(pwm_pin , abs(PWM_vel) );
    digitalWrite(inpA , HIGH);
    digitalWrite(inpB , LOW);
  }
  else{
    analogWrite(pwm_pin , abs(PWM_vel) );
    digitalWrite(inpA , LOW);
    digitalWrite(inpB , HIGH);
  }
}

// Custom servo motor contorl function
void servo_write(int degree, int pin){

  degree = constrain(degree,min_servo_pos,max_servo_pos);
  int val = (degree*10.25)+500;
  int cycles = 5;
  
  for(int i=0; i< cycles; i++){
    digitalWrite(pin,HIGH);
    delayMicroseconds(val);
    digitalWrite(pin,LOW);
    delay(10);
  }
}

// encoders interrupts
void read_encoder () {
  if (digitalRead(encoder_PinB) > 0){
    encoder_pos++;
  }
  else {
    encoder_pos--;
  }
}

    /////////////fruit classification////////////////
   // BY : Abdelraouf Hawash                 //
  // DATE : 8/4/2023                      //
 // GMAIL : abdelraouf.hawash@gmail.come   //
////////////////////////////////////////////



///// initializing hardware variables
const int pwm_motor_pin = 10; // motor pins
const int inA_motor_pin = 8 ;  
const int inB_motor_pin = 9 ;
#define encoder_PinA 3        //encoder chanals
#define encoder_PinB 5
const int servo_pin = 6;      // servo pin (we will control servo manualy to use anther pin as PWM)

//// variables
//motor
int max_pwm = 255;            //maximum pwm command for motors
double target_speed = 0;      //goal linear speed for the belt, in cm/s
double current_speed = 0;     //Actual linear speed for the belt in cm/s
//servo
double target_servo_pos = 90; // servo goal pulses
const int max_servo_pos = 150;// maximum pulses for servo
const int min_servo_pos = 40; // minimum pulses for servo
// time variables
const int loop_time = 300 ;       //Looptime in millisecond
unsigned long lastMilli = 0;      //time history
const byte noCommLoopMax = 10;    //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;     //main loop without communication counter
// encoder
const double encoder_Puls_per_cm = 62.5; // encoder puls per cm
double encoder_pulses_in_cmps = (encoder_Puls_per_cm/1000) * loop_time; //pulses that gives speed 1cm/s within loop time(ms)
// initializing PID
double Kp = 0;
double Ki = 0;
double Kd = 0;
unsigned long currentTime, previousTime;
double elapsedTime, error, lastError, cumError, rateError;
volatile float current_encoder_pulses = 0; //encoder pulses counter
double target_encoder_pulses = 0;          // goal pulses
double speed_pwm = 0 ;                     //PWM commond for the motor

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

  // read and write data
  if (Serial.available()){
    // read
    String Data_cmd = Serial.readString();
    // update cmd
    update_cmd(Data_cmd);
    // write
    Serial.print(String(current_speed));
    // Reset the counter for number of main loops without communication
    noCommLoops = 0;
  }

  if((millis()-lastMilli) >= loop_time){
    // record time
    lastMilli = millis();
    
    // get current speed pulses
    if (abs(current_encoder_pulses) < 2) current_speed = 0;     //Avoid taking in account small disturbances
    else current_speed = current_encoder_pulses / encoder_pulses_in_cmps ; // calculate speed of the belt in cm/s  
    
    //Calculate the target pulses to give required speed
    target_encoder_pulses = target_speed * encoder_pulses_in_cmps;  // in cm
    
    // motor action with PID
    computePID();
    set_motor(round(speed_pwm),pwm_motor_pin,inA_motor_pin,inB_motor_pin);
    
    // motor action without PID
    // set_motor(50*target_speed,pwm_motor_pin,inA_motor_pin,inB_motor_pin);

    // servo action
    servo_write(target_servo_pos,servo_pin);

    // reset encoder pulses to zero
    current_encoder_pulses = 0;
    
    noCommLoops++;
    
    if (noCommLoops >= noCommLoopMax) {      //Stopping if too much time without command or requested speed equal zero
      target_speed = 0 ;
      
      if (noCommLoops == 65535){
        noCommLoops = noCommLoopMax;
      }
    }

    // Serial.println(current_speed); // for monitring on serial for test
    // Serial.println(round(speed_pwm));
    
  } // end of timed loop
} // end of main loop



//update commands received like requested pulses and other
void update_cmd(String input){
  
  // input should be : target_speed,target_servo_pos,max_pwm,kp,ki,kd
  char separator = ',';
  int maxIndex = input.length() - 1;
  int cmd_index = 0;

  String Value = "";
  
  for(int i= 0 ; i <= maxIndex ; i++){
    
    Value.concat(input[i]);
    
    if (input[i] == separator || i == maxIndex)
    {
        Value.trim();

        if (cmd_index == 0) target_speed = Value.toDouble(); 
        if (cmd_index == 1) target_servo_pos = constrain(Value.toDouble(), min_servo_pos , max_servo_pos);
        if (cmd_index == 2) Kp = Value.toDouble();
        if (cmd_index == 3) Ki = Value.toDouble();
        if (cmd_index == 4) Kd = Value.toDouble();
        if (cmd_index == 5) max_pwm = constrain(Value.toInt(), 0 , 255);
        
        cmd_index++;
        Value = "";
    }
  }
}

// PID control function
void computePID(){

  currentTime = millis();                             //get current time
  elapsedTime = (double)(currentTime - previousTime); //compute time elapsed from previous computation
  error = target_encoder_pulses - current_encoder_pulses;  // determine error
  cumError += error * elapsedTime;                    // compute integral
  rateError = (error - lastError)/elapsedTime;        //compute derivative
  double output = Kp*error + Ki*cumError + Kd*rateError;     //PID output
  speed_pwm = constrain(output,-max_pwm,max_pwm);
  
  previousTime = currentTime; //remember current time
  lastError = error;          //remember current error
  
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
    current_encoder_pulses++;
  }
  else {
    current_encoder_pulses--;
  }
}

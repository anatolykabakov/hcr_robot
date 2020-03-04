

int serial_port_rate = 57600;

const byte encoderRpinA = 3;                              //A pin -> the interrupt pin (2)
const byte encoderLpinA = 2;                              //A pin -> the interrupt pin (3)

const int MotorRpwm = 5; // 
const int MotorLpwm = 6;
const int MotorLdir = 7;
const int MotorRdir = 4;

//
//const byte encoderRpinA = 2;        //A pin -> the interrupt pin (2)
//const byte encoderRpinB = 17;       //B pin -> the digital pin (16)
//const byte encoderLpinA = 3;        //A pin -> the interrupt pin (3)
//const byte encoderLpinB = 16;       //B pin -> the digital pin (17)
//
//const int MotorRdir = 52;    //Right motor Direction Control pin
//const int MotorLdir = 53;    //Left motor Direction Control pin
//const int MotorRpwm = 4;     //Right motor PWM Speed Control pin
//const int MotorLpwm = 5;     //Left motor PWM Speed Control pin

int ticks_counter =0;
int set_ticks = 10000;

unsigned long wheelImpR = 0; // число импульсов с энкодера правого колеса 
unsigned long wheelImpL = 0; // число импульсов с энкодера левого колеса 

int pwm_right = 120;
int pwm_left = 120;

void setup() {
  Serial.begin(serial_port_rate);//Initialize the Serial port
  pinMode(MotorRdir, OUTPUT);
  pinMode(MotorLdir, OUTPUT);
  EncoderInit();

}

void loop() {
  Serial.print("pp");Serial.println();

  if (set_ticks > wheelImpR) {
    digitalWrite(MotorRdir,LOW); analogWrite(MotorRpwm, pwm_right);
  }
  else{
    digitalWrite(MotorRdir,LOW); analogWrite(MotorRpwm, 0);
  }
  
  if (set_ticks > wheelImpR) {
    digitalWrite(MotorLdir,HIGH); analogWrite(MotorLpwm,pwm_left);
  }
  else{
    digitalWrite(MotorLdir,LOW); analogWrite(MotorLpwm,0);
  }
  
}

void EncoderInit()
{
  pinMode(encoderRpinA,INPUT); 
  pinMode(encoderLpinA,INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderRpinA), WheelPulseR, RISING ); 
  attachInterrupt(digitalPinToInterrupt(encoderLpinA), WheelPulseL, RISING );  
}

void WheelPulseR(){  
  wheelImpR ++;
}

void WheelPulseL(){   
  wheelImpL ++; 
}

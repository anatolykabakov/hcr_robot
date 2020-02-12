#include <math.h>
#include <string.h>

// -------------------------------------------------------------------------
//
// variables for black robot
double R = 0.0682; // meters radius
double L = 0.29;  // meters wheel dist  

double maxSpeed = 0.47; // максимальная линейная скорость при скважности 100%, в м/с

//arduino pins 
const byte encoderRpinA = 3;                              //A pin -> the interrupt pin (2)
const byte encoderLpinA = 2;                              //A pin -> the interrupt pin (3)

const int MotorRpwm = 5; // 
const int MotorLpwm = 6;
const int MotorLdir = 7;
const int MotorRdir = 4;

//Resolution encoders 
int resolution_encoders = 1435; //

//PID variables
double Kp = 1.6;
double Ki = 11.6;

// -------------------------------------------------------------------------
/// variables for HCR
// double R = 0.0682; // meters radius
// double L = 0.275;  // meters wheel dist 
 
// int resolution_encoders = 663; // 

// double maxSpeed = 0.7; // максимальная линейная скорость при скважности 100%, в м/с

// const byte encoderRpinA = 2;        //A pin -> the interrupt pin (2)
// const byte encoderRpinB = 17;       //B pin -> the digital pin (16)
// const byte encoderLpinA = 3;        //A pin -> the interrupt pin (3)
// const byte encoderLpinB = 16;       //B pin -> the digital pin (17)

// const int MotorRdir = 52;    //Right motor Direction Control pin
// const int MotorLdir = 53;    //Left motor Direction Control pin
// const int MotorRpwm = 4;     //Right motor PWM Speed Control pin
// const int MotorLpwm = 5;     //Left motor PWM Speed Control pin


// // //PID variables
// // double Kp = 0.4;
// // double Ki = 10;

// double Kp = 0.83;
// double Ki = 16.0;

// -------------------------------------------------------------------------

unsigned long wheelImpR = 0; // число импульсов с энкодера правого колеса 
unsigned long wheelImpL = 0; // число импульсов с энкодера левого колеса 

const int rate = 20; //hz
double update_interval = 1/double(rate); // Sec
const int arduino_delay= int(update_interval*1000); // mSec

bool is_connected = false;

double SetSpeedR = 0;   //Wish Speed of right motor
double SetSpeedL = 0;   //Wish Speed of left motor

double wheelRightV = 0;
double wheelLeftV = 0;
double linear_velocity = 0;
double angular_velocity = 0;

double integral_right = 0;
double integral_left = 0;

bool DirectionR = true;
bool DirectionL = true;

struct Pose{
  double x = 0; // meters
  double y = 0;
  double yaw = 0; // rad
  double v = 0; // metert per second
  double w = 0; // rad per second
};

Pose robot;

void setup()
{
  Serial.begin(57600);//Initialize the Serial port
  pinMode(MotorRdir, OUTPUT);
  pinMode(MotorLdir, OUTPUT);
  EncoderInit();
  
}

void loop()
{
  // --------------- Подсчет информации с энкодеров --------------------
  CalcEncNav();
  // --------------- Чтение уставок по линейной и угловой скорости --------------------
  get_messages_from_Serial();
  // --------------- Расчет ПИД  --------------------
  
  double speed_pid_left = PI_Controller_Left(SetSpeedL, wheelLeftV);
  double speed_pid_right = PI_Controller_Right(SetSpeedR, wheelRightV);
  // --------------- Подача уставок на двигатели  --------------------
  move(speed_pid_left, speed_pid_right);
//  move(Set/SpeedL, SetSpeedR);
  
  delay(arduino_delay); 
} 

void move(double set_speed_left, double set_speed_right)
{

  if (set_speed_left > maxSpeed) set_speed_left = maxSpeed;
  if (set_speed_right > maxSpeed) set_speed_right = maxSpeed;

  
  int pwm_left = (set_speed_left / maxSpeed) * 255;
  int pwm_right = (set_speed_right / maxSpeed) * 255;
  
  if (pwm_left < 20) pwm_left = 0;
  if (pwm_right < 20) pwm_right = 0;
  if (pwm_left >= 255) pwm_left = 255;
  if (pwm_right > 255) pwm_right = 255;

  if (DirectionR){digitalWrite(MotorRdir,LOW); analogWrite(MotorRpwm, pwm_right);}
  else{digitalWrite(MotorRdir,HIGH); analogWrite(MotorRpwm, pwm_right);}
  
  // FOR black robot
  if (DirectionL){digitalWrite(MotorLdir,HIGH); analogWrite(MotorLpwm,pwm_left);}
  else{digitalWrite(MotorLdir,LOW); analogWrite(MotorLpwm,pwm_left);}
  
  // FOR HCR

//  if (DirectionL){digitalWrite(MotorLdir,LOW); analogWrite(MotorLpwm,pwm_left);}
//  else{digitalWrite(MotorLdir,HIGH); analogWrite(MotorLpwm,pwm_left);}

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

void CalcEncNav()  {
  // пройденный колесом путь, м
  double wheelRightS = ((double(wheelImpR) / resolution_encoders) * 2 * M_PI *  R) ; // метры L = 2*PI*R*n/N 
  double wheelLeftS  = ((double(wheelImpL) / resolution_encoders) * 2 * M_PI *  R); //* 

  // обнулем импульсы
  wheelImpR = 0;
  wheelImpL = 0;
  
  // линейная скорость колеса
  wheelRightV = double(wheelRightS/ update_interval); // mетры за сек
  wheelLeftV  = double(wheelLeftS / update_interval);

  // #odometry navigation 
  // # http://faculty.salina.k-state.edu/tim/robotics_sg/Control/kinematics/unicycle.html
  double omegaRight = wheelRightV/R; // rad/s
  double omegaLeft  = wheelLeftV/R;

  omegaLeft *= DirectionL == true ? 1 : -1;
  omegaRight *= DirectionR == true ? 1 : -1;

  robot.v = (R/2)*(omegaRight + omegaLeft);
  robot.w = (R/L)*(omegaRight - omegaLeft);

  robot.yaw+=(robot.w * update_interval); // rad
  robot.yaw = normalize_angle(robot.yaw);
  robot.x += robot.v*cos(robot.yaw) * update_interval;
  robot.y += robot.v*sin(robot.yaw) * update_interval;

}

double normalize_angle(double angle){
    // """
    // Normalize an angle to [-pi, pi].

    // :param angle: (float)
    // :return: (float) Angle in radian in [-pi, pi]
    // """
    while (angle > M_PI)
    {
      angle -= 2.0 * M_PI;
    }
        
    while (angle < -M_PI)
    {
        angle += 2.0 * M_PI;
    }

    return angle;
}

double PI_Controller_Left(double set_speed, double current_speed)
{
  
  double errorL = set_speed -  current_speed;

  integral_left = integral_left + (errorL*update_interval);
  
  double speed_pid_left = Kp*errorL + Ki*integral_left;

  return speed_pid_left;
    
}

double PI_Controller_Right(double set_speed, double current_speed)
{
  //http://robotsforroboticists.com/pid-control/
  double errorR = set_speed -  current_speed;

  integral_right = integral_right + (errorR*update_interval);
  
  double speed_pid_right = Kp*errorR + Ki*integral_right;
  
  return speed_pid_right;
  
}

void get_messages_from_Serial()
{
  if(Serial.available() > 0)
  {
    // The first byte received is the instruction
    int order_received = Serial.read();

    if(order_received == 's')
    {
      // If the cards haven't say hello, check the connection
      if(!is_connected)
      {
        is_connected = true;
        Serial.print("r");
      }
    }
    else
    {
      switch(order_received)
      {

        case 'v'://если v, то считываем уставку по скорости
        {
          char buffer[12];
          String line = Serial.readStringUntil('\n');// считываем скорости для левого и правого колеса [40 50]
          line.toCharArray(buffer,12);//переводим в char
          double vLinear = atof(strtok(buffer," "));//разделяем на скорости левого и правого колеса
          double vAngular = atof(strtok(NULL,  " "));
          // v=ωR. 
          SetSpeedR = ((2 * vLinear) + (L * vAngular))/2; // M/SEC
          SetSpeedL = ((2 * vLinear) - (L * vAngular))/2;

          //определяем напрвление врщения колес по знаку      
          DirectionL = SetSpeedL > 0 ? true : false;
          DirectionR = SetSpeedR > 0 ? true : false;
          
          SetSpeedR = abs(SetSpeedR);
          SetSpeedL = abs(SetSpeedL);

          break;
        }
        case 'd'://если d, то печатаем текущие значения
        {          
          if(robot.x >= 0){Serial.print ("+");Serial.print (abs(robot.x)); Serial.print (";");}
          else {Serial.print ("-");Serial.print (abs(robot.x)); Serial.print (";");}
          
          if(robot.y >= 0){Serial.print ("+");Serial.print (abs(robot.y)); Serial.print (";");}
          else {Serial.print ("-");Serial.print (abs(robot.y)); Serial.print (";");}
          
          if(robot.yaw >= 0){Serial.print ("+");Serial.print (abs(robot.yaw)); Serial.print (";");}
          else {Serial.print ("-");Serial.print (abs(robot.yaw)); Serial.print (";");}

          if(robot.v >= 0){Serial.print ("+");Serial.print (abs(robot.v)); Serial.print (";");}
          else {Serial.print ("-");Serial.print (abs(robot.v)); Serial.print (";");}

          if(robot.w >= 0){Serial.print ("+");Serial.print (abs(robot.w)); Serial.print (";");}
          else {Serial.print ("-");Serial.print (abs(robot.w)); Serial.print (";");}

          if(DirectionR == false){Serial.print ("-");Serial.print (abs(wheelRightV)); Serial.print (";");}
          else {Serial.print ("+");Serial.print (abs(wheelRightV)); Serial.print (";");}

          if(DirectionL == false){Serial.print ("-");Serial.print (abs(wheelLeftV)); Serial.print (";");}
          else {Serial.print ("+");Serial.print (abs(wheelLeftV)); Serial.print (";");}
          
          break;
        }
        default:

          return;
      }
    }
    Serial.print("c");
 
  }
}

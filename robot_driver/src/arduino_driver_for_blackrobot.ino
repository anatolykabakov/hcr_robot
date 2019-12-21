int E1 = 5;
int M1 = 4;
int E2 = 6;
int M2 = 7;


const byte encoderRpinA = 2;                              //A pin -> the interrupt pin (2)
const byte encoderLpinA = 3;                              //A pin -> the interrupt pin (3)

unsigned long wheelImpR = 0; // число импульсов с энкодера правого колеса 
unsigned long wheelImpL = 0; // число импульсов с энкодера левого колеса 
const int Interval=10;
double dT = double(Interval)/1000;

double wheelLeftS = 0;
double wheelRightS = 0;
double wheelLeftV = 0;
double wheelRightV = 0;
double maxSpeed = 0.5; // максимальная линейная скорость при скважности 100%, в м/с
double wheelSpeedR = 0;  // Скорость правого колеса с энкодера
double wheelSpeedL = 0;  // Скорость левого колеса с энкодера
double actualSpeedR = 0;
double actualSpeedL = 0;


bool is_connected = false;
int rightPWM = 0;
int leftPWM = 0;

// HCR parameters 
double R = 0.0682; // meters radius
double L = 0.275;  // meters wheel dist  
int resolution_encoders = 1440; // 20000 / 14

double SetSpeedR = 0;   //Wish Speed of right motor
double SetSpeedL = 0;   //Wish Speed of left motor

//PID variables
double Kp = 1;
double Ki = 2.5;
double SetSpeedControllerR = 0;
double SetSpeedControllerL = 0;
double speed_pid_left = 0;
double speed_pid_right = 0;
double errorR=0;
double errorL=0;
double integral_left = 0;
double integral_right = 0;

int count = 20000;

bool DirectionR = true;
bool DirectionL = true;

#include <string.h>
char buffer[10];

void setup()
{
  Serial.begin(115200);//Initialize the Serial port
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  EncoderInit();
}

void loop()
{
  // --------------- Подсчет информации с энкодеров --------------------
  CalcEncNav();
  // --------------- Чтение уставок по линейной и угловой скорости --------------------
  get_messages_from_Serial();
  // --------------- Расчет ПИД  --------------------
  PI_Controller_Left();
  PI_Controller_Right();
  // --------------- Подача уставок на двигатели  --------------------
  move(speed_pid_left, speed_pid_right);
  //move(SetSpeedL, SetSpeedR);
  
  delay(Interval);  
  
} 

void move(float speed_left, float speed_right)
{
  
  int pwm_left = speed_left;//(speed_left / maxSpeed) * 255;
  int pwm_right = speed_right;//(speed_right / maxSpeed) * 255;
  if (pwm_left < 20) pwm_left = 0;
  if (pwm_right < 20) pwm_right = 0;
  if (pwm_left >= 255) pwm_left = 255;
  if (pwm_right > 255) pwm_right = 255;
  //Serial.print(speed_left);Serial.print(" ");Serial.print(speed_right);Serial.println();
  if (DirectionR){digitalWrite(M1,LOW); analogWrite(E1, pwm_right);}
  else{digitalWrite(M1,HIGH); analogWrite(E1, pwm_right);}
  
  if (DirectionL){digitalWrite(M2,HIGH); analogWrite(E2,pwm_left);}
  else{digitalWrite(M2,LOW); analogWrite(E2,pwm_left);}

}

void EncoderInit()

{
  pinMode(encoderRpinA,INPUT); 
  pinMode(encoderLpinA,INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderRpinA), WheelPulseL, RISING ); // вызов процедуры по прерыванию. Параметры: номер прерывания (не ножки), имя процедуры, состояние сигнала
  attachInterrupt(digitalPinToInterrupt(encoderLpinA), WheelPulseR, RISING );  // ЗАМЕНА, была ссылка на DecodeSpeedL
}

void WheelPulseR(){   // Счетчик спиц правого колеса 
  wheelImpR ++;
}
void WheelPulseL(){   // Счетчик спиц левого колеса 
  wheelImpL ++;
}

void CalcEncNav()  {
  wheelSpeedR = double(wheelImpR / dT); // число импульсов за сек
  wheelSpeedL = double(wheelImpL / dT); // число импульсов за сек

  // пройденный колесом путь, м
  wheelRightS = ((wheelSpeedR / resolution_encoders) * 2 * 3.14 *  R) ; // метры L = 2*PI*R*n/N 
  wheelLeftS  = ((wheelSpeedL / resolution_encoders) * 2 * 3.14 *  R); //* 
  
  // линейная скорость колеса
  wheelRightV = wheelRightS/ 1; // mетры за сек
  wheelLeftV  = wheelLeftS / 1;

  // обнулем импульсы
  wheelImpR = 0;
  wheelImpL = 0;

  actualSpeedR= wheelRightV * 255 / maxSpeed;           // обратная связь ПИД-регулятора, м/сек
  actualSpeedL= wheelLeftV * 255 / maxSpeed;

}


void PI_Controller_Left()
{
  //Serial.print(errorL);Serial.print(" ");Serial.print(SetSpeedControllerL);Serial.print(" ");Serial.println(actualSpeedL);
  errorL = SetSpeedControllerL - actualSpeedL;
  if (errorL <= 0)
  {
    speed_pid_left = 0;
  }
  else
  {
    integral_left = integral_left + (errorL*dT);
    speed_pid_left = Kp*errorL + Ki*integral_left;
  }

}

void PI_Controller_Right()
{
  //http://robotsforroboticists.com/pid-control/
  errorR = SetSpeedControllerR -  actualSpeedR;
  if (errorR <= 0)
  {
    speed_pid_right = 0;
  }
  else
  {
    integral_right = integral_right + (errorR*dT);
    speed_pid_right = Kp*errorR + Ki*integral_right;
  }
  
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
          String line = Serial.readStringUntil('\n');// считываем скорости для левого и правого колеса [40 50]
          line.toCharArray(buffer,10);//переводим в char
          SetSpeedR = atof(strtok(buffer," "));//разделяем на скорости левого и правого колеса
          SetSpeedL = atof(strtok(NULL,  " "));

          //определяем напрвление врщения колес по знаку
          if (SetSpeedR > 0) {SetSpeedR = abs(SetSpeedR);DirectionR = true;}
          else {SetSpeedR = abs(SetSpeedR);DirectionR = false;}
          if (SetSpeedL > 0) {SetSpeedL = abs(SetSpeedL);DirectionL = true;}
          else {SetSpeedL = abs(SetSpeedL);DirectionL = false;}
          SetSpeedControllerR= (SetSpeedR/maxSpeed) * 255 ; // уставка скорости
          SetSpeedControllerL= (SetSpeedL/maxSpeed) * 255;
          
          break;
        }
        case 'd'://если d, то печатаем текущие значения
        {
           // определяем знак по направлению вращения
          
          if(DirectionR == false){Serial.print ("-");Serial.print (wheelRightV); Serial.print (";");}
          else {Serial.print ("+");Serial.print (wheelRightV); Serial.print (";");}
          if(DirectionL == false){Serial.print ("-");Serial.print (wheelLeftV); Serial.print (";");}
          else {Serial.print ("+");Serial.print (wheelLeftV); Serial.print (";");}
          
          break;
        }
        // Unknown order
        default:

          return;
      }
    }
    Serial.print("c");
 
  }
}
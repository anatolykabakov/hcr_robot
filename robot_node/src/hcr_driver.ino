//Использование выводов
/*
===PWM==============================================================
Моторы
4 ---> правый мотор PWM1 (+ земля)
5 ---> левый мотор PWM2


===DIGITAL==========================================================
Моторы
52 ---> Direction-пин правого мотора DIR1
53 ---> Direction-пин левого мотора DIR2




#include <Metro.h>
#include <PID_v1.h>
#include <Math.h>
#include <TimerOne.h> // http://www.arduino.cc/playground/Code/Timer1
#include <Wire.h>

// #include <Check_times.cpp>


// MegaADK DIGITAL PINS USABLE FOR INTERRUPTS 2, 3, 18, 19, 20, 21
//                                                 I2C pins 20, 21

//Encoder variables

const byte encoderRpinA = 2;                              //A pin -> the interrupt pin (2)
const byte encoderRpinB = 17;                              //B pin -> the digital pin (16)
const byte encoderLpinA = 3;                              //A pin -> the interrupt pin (3)
const byte encoderLpinB = 16;                              //B pin -> the digital pin (17)


//Motor control variables
const int MotorRdir = 52;    //Right motor Direction Control pin
const int MotorLdir = 53;    //Left motor Direction Control pin
const int MotorRpwm = 4;     //Right motor PWM Speed Control pin
const int MotorLpwm = 5;     //Left motor PWM Speed Control pin
bool DirectionR = 0;     //Right Motor Direction
bool DirectionL = 0;     //Left Motor Direction

double LinearVelocity = 0;
double AngularVelocity = 0;
double wheelSpeedR = 0;  // Скорость правого колеса с энкодера
double wheelSpeedL = 0;  // Скорость левого колеса с энкодера
unsigned long wheelImpR = 0; // число импульсов с энкодера правого колеса 
unsigned long wheelImpL = 0; // число импульсов с энкодера левого колеса 
unsigned long EncodersCounterR = 0; // число импульсов с энкодера правого колеса 
unsigned long EncodersCounterL = 0; // число импульсов с энкодера левого колеса 

//PID variables
double Kp = 0.4;
double Ki = 10;
double SetSpeedControllerR = 0;
double SetSpeedControllerL = 0;
double actualSpeedR = 0;
double actualSpeedL = 0;
double outputL = 0;
double outputR = 0;
double errorR=0;
double errorL=0;
double integral_left = 0;
double integral_right = 0;

const int Interval=10; 
double dT = double(Interval)/1000;           // 10 ms период счета 

double SetSpeedR = 0;   //Wish Speed of right motor
double SetSpeedL = 0;   //Wish Speed of left motor

//------------------------------------------------
#include <string.h>
char buffer[10];
double wheelLeftS = 0;
double wheelRightS = 0;
double wheelLeftV = 0;
double wheelRightV = 0;
double maxSpeed = 0.94; // максимальная линейная скорость при скважности 100%, в м/с
double maxSpeedR = 0.67;
double maxSpeedL = 0.94;

bool is_connected = false;
int rightPWM = 0;
int leftPWM = 0;

// HCR parameters 
double R = 0.0682; // meters radius
double L = 0.275;  // meters wheel dist  



void setup() {
   Init();
}
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Главный цикл ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
void loop() { 
  // --------------- Подсчет информации с энкодеров --------------------
  CalcEncNav();
  // --------------- Чтение уставок по линейной и угловой скорости --------------------
  get_messages_from_Serial();
  // --------------- Расчет ПИД  --------------------
  PI_Controller_Left();
  PI_Controller_Right();
  // --------------- Подача уставок на двигатели  --------------------
  Movement();
  //Movement_noPID(SetSpeedR,SetSpeedL);
  // --------------- Задержка для обновления энкодеров  --------------------
  delay(Interval);     
  //delay some certain time for aquiring pulse from encoder
 }
 //loop ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void Init(){
  Wire.begin();
  Serial.begin(57600);//Initialize the Serial port
  while (!Serial) ; // while the Serial stream is not open, do nothing
  MotorsInit();
  EncoderInit();//Initialize encoder 
}

void MotorsInit() { //Initialize motors variables 
  DirectionR = LOW;
  DirectionL = LOW;
  SetSpeedR = 0;
  SetSpeedL = 0;
  
  pinMode(MotorRdir, OUTPUT);
  pinMode(MotorLdir, OUTPUT);
  pinMode(MotorRpwm, OUTPUT);
  pinMode(MotorLpwm, OUTPUT);
  
  digitalWrite (MotorRdir, DirectionR);
  digitalWrite (MotorLdir, DirectionL);
  analogWrite (MotorRpwm, SetSpeedR);
  analogWrite (MotorLpwm, SetSpeedL);
}
void EncoderInit() { //Initialize encoder interruption 

  pinMode(encoderRpinA,INPUT);  // Right weel
  pinMode(encoderRpinB,INPUT);  
  pinMode(encoderLpinA,INPUT);  // Left weel
  pinMode(encoderLpinB,INPUT);  
 
  // Привязка прерывания по импульсу энкодера
  attachInterrupt(digitalPinToInterrupt(encoderRpinA), WheelPulseR, RISING ); // вызов процедуры по прерыванию. Параметры: номер прерывания (не ножки), имя процедуры, состояние сигнала
  attachInterrupt(digitalPinToInterrupt(encoderLpinA), WheelPulseL, RISING );  // ЗАМЕНА, была ссылка на DecodeSpeedL
 
}
void WheelPulseR(){   // Счетчик спиц правого колеса 
  wheelImpR ++;
  EncodersCounterR++;
}
void WheelPulseL(){   // Счетчик спиц левого колеса 
  wheelImpL ++;
  EncodersCounterL++;
}

void PI_Controller_Left()
{
  errorL = SetSpeedControllerL - actualSpeedL;
  integral_left = integral_left + (errorL*dT);
  outputL = Kp*errorL + Ki*integral_left;

}
void PI_Controller_Right()
{
  //http://robotsforroboticists.com/pid-control/
  errorR = SetSpeedControllerR -  actualSpeedR;
  integral_right = integral_right + (errorR*dT);
  outputR = Kp*errorR + Ki*integral_right;

}
void Movement(){//move
  rightPWM = int(outputR); 
  leftPWM = int(outputL);
  
  //ограничение 13..255
  if (rightPWM >= 255){rightPWM=255;}
  if (rightPWM < 13) {rightPWM = 0;}
  if (leftPWM >= 255){leftPWM=255;}
  if (leftPWM < 13) {leftPWM = 0;}
  
  analogWrite (MotorRpwm,rightPWM);      //motor1 move forward at speed a
  digitalWrite(MotorRdir,DirectionR);  
  analogWrite (MotorLpwm,leftPWM);      //motor2 move forward at speed b
  digitalWrite(MotorLdir,DirectionL);  
}
void CalcEncNav()  {
  wheelSpeedR = double(wheelImpR / dT); // число импульсов за сек
  wheelSpeedL = double(wheelImpL / dT); // число импульсов за сек

  // пройденный колесом путь, м
  wheelRightS = ((wheelSpeedR / 663) * 2 * 3.14 *  R) ; // метры L = 2*PI*R*n/N 
  wheelLeftS  = ((wheelSpeedL / 663) * 2 * 3.14 *  R); //* 
  
  // линейная скорость колеса
  wheelRightV = wheelRightS/ 1; // mетры за сек
  wheelLeftV  = wheelLeftS / 1;

  // обнулем импульсы
  wheelImpR = 0;
  wheelImpL = 0;

  actualSpeedR= wheelRightV * 255 / maxSpeed;           // обратная связь ПИД-регулятора, м/сек
  actualSpeedL= wheelLeftV * 255 / maxSpeed;

}

void Movement_noPID(double rightVelOut,double leftVelOut){//move
  // Проверка направления вращения
  if (rightVelOut< 0) {rightVelOut = abs(rightVelOut); DirectionR = HIGH;}
  else {DirectionR = LOW;}
  if (leftVelOut < 0) {leftVelOut = abs(leftVelOut);   DirectionL = HIGH;}
  else {DirectionL = LOW;}
  // Переход от метров в сек к ШИМ 0..255
  rightPWM= int(rightVelOut * 255 /maxSpeed); // уставка скорости
  leftPWM = int(leftVelOut * 255 /maxSpeed);
  // Ограничения 13..255
  if (rightPWM < 13) {rightPWM = 0;}
  if (leftPWM < 13) {leftPWM = 0;}
  if (rightPWM >= 255) {rightPWM = 255;}
  if (leftPWM >= 255) {leftPWM = 255;}
  // Подаем на двигатели ШИМ 13..255
  analogWrite (MotorRpwm,rightPWM);      //motor1 move forward at speed a
  digitalWrite(MotorRdir,DirectionR);  
  analogWrite (MotorLpwm,leftPWM);      //motor2 move forward at speed b
  digitalWrite(MotorLdir,DirectionL);  
}

void get_messages_from_Serial()
{
  if(Serial.available() > 0)
  {
    // The first byte received is the instruction
    int order_received = Serial.read();

    switch(order_received)
    {
      case 'v'://если v, то считываем уставку по скорости
      {
        String line = Serial.readStringUntil('\n');// считываем скорости для левого и правого колеса [40 50]
        line.toCharArray(buffer,10);//переводим в char
        SetSpeedR        = atof(strtok(buffer," "));//разделяем на скорости левого и правого колеса
        SetSpeedL       = atof(strtok(NULL,  " "));
        //определяем напрвление врщения колес по знаку
        if (SetSpeedR < 0) {SetSpeedR = abs(SetSpeedR); DirectionR = true;}
        else {DirectionR = false;}
        if (SetSpeedL < 0) {SetSpeedL = abs(SetSpeedL); DirectionL = true;}
        else {DirectionL = false;}
        SetSpeedControllerR= (SetSpeedR * 255 /maxSpeed); // уставка скорости
        SetSpeedControllerL= (SetSpeedL * 255 /maxSpeed);
        
        break;
      }
      case 'd'://если d, то печатаем текущие значения
      {
         // определяем знак по направлению вращения
        Serial.print (EncodersCounterR);
        Serial.print (EncodersCounterL);
        Serial.print("c");
        break;
      }
      // Unknown order
      default:

        return;
    }
  }
  
}
      

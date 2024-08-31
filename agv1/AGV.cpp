#include "AGV.h"

//*******Driver Class***********
Driver::Driver(byte R_EN, byte L_EN, byte RPWM, byte LPWM)
{
  this->R_EN = R_EN;
  this->L_EN = L_EN;
  this->RPWM = RPWM;
  this->LPWM = LPWM;
}

void Driver::init()
{
  pinMode(R_EN,OUTPUT);
  pinMode(L_EN,OUTPUT);
  pinMode(RPWM,OUTPUT);
  pinMode(LPWM,OUTPUT);

  digitalWrite(R_EN, LOW);
  digitalWrite(L_EN, LOW);

  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);


}

byte Driver::forward(int speed)
{
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);

  analogWrite(LPWM, 0);
  analogWrite(RPWM,speed);
  direction = 1;
  return direction;
}

byte Driver::backward(int speed)
{
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);

  analogWrite(LPWM, speed);
  analogWrite(RPWM,0);

  direction = 0;
  return direction;
}

  void Driver::up()
  {
    digitalWrite(R_EN, HIGH);
    digitalWrite(L_EN, HIGH);

    analogWrite(LPWM, 255);
    analogWrite(RPWM,0);
  }
  void Driver::down()
  {
    digitalWrite(R_EN, HIGH);
    digitalWrite(L_EN, HIGH);

    analogWrite(LPWM, 0);
    analogWrite(RPWM,255);
  }

  void Driver::stop()
  { /*
    switch (direction) 
    {
    case 1:
    analogWrite(LPWM, 0);
    analogWrite(RPWM,speed);
    break;
    case 0:
    analogWrite(LPWM, speed);
    analogWrite(RPWM,0);
    break;
    }*/
        
    digitalWrite(R_EN, LOW);
    digitalWrite(L_EN, LOW);
  }

//********Robot Class**************** 
Robot::Robot(Driver &right_motor, Driver &left_motor)
{
  this->right_motor = right_motor;
  this->left_motor = left_motor;
}

Robot::Robot(Driver &right_motor, Driver &left_motor, Driver actuator1, Driver actuator2)
{
  this->right_motor = right_motor;
  this->left_motor = left_motor;
  
  this->actuator1 = actuator1;
  this->actuator2 = actuator2;
}

void Robot::init()
{
  right_motor.init();
  left_motor.init();

  actuator1.init();
  actuator2.init();
}
void Robot::start_move(byte direction, int speed)
{ 
  switch (direction)
  {
    case FORWARD:
    for(int i = 0; i <= speed; i++)
    {
      forward(speed,speed);
      delay(10);
    }
    forward(speed,speed);
    break;
    case BACKWARD:
    for(int i = 0; i <= speed; i++)
    {
      backward(speed,speed);
      delay(10);
    }
    backward(speed, speed);
    break;
  }
}
void Robot::forward(int right_speed, int left_speed)
{
  right_motor.forward(right_speed);
  left_motor.forward(left_speed);
}
void Robot::backward(int right_speed, int left_speed)
{
  right_motor.backward(right_speed);
  left_motor.backward(left_speed);
}

void Robot::up()
{
  actuator1.up();
  actuator2.up();
}
void Robot::down()
{
  actuator1.down();
  actuator2.down();
}

void Robot::stop_lift()
{
  actuator1.stop();
  actuator2.stop();
}

void Robot::stop_move()
{
  right_motor.stop();
  left_motor.stop();
}

void Robot::right(int right_speed, int left_speed)
{
  right_motor.backward(right_speed);
  left_motor.forward(left_speed);
}

void Robot::left(int right_speed, int left_speed)
{
  right_motor.forward(right_speed);
  left_motor.backward(left_speed);
}
//*******Encoder***********
#include <util/atomic.h> // For the ATOMIC_BLOCK macro

Encoder* Encoder::sEncoder = 0;
Encoder::Encoder(byte ENA, byte ENB)
{ 
  sEncoder = this;
  this->ENA = ENA;
  this->ENB = ENB;
}
void Encoder::init()
{ 
  pinMode(ENA,INPUT_PULLUP);
  pinMode(ENB,INPUT_PULLUP);
  //call pulse() when any high/low changed seen
  attachInterrupt(digitalPinToInterrupt(ENA), updateEncoderISR, RISING);
}

void Encoder::pulse()
{
  int b = digitalRead(ENB);
  if(b > 0){
    ticks++;
  }
  else{
    ticks--;
  }
}

void Encoder::updateEncoderISR()
{
    if (sEncoder != 0)
        sEncoder->pulse();
}
int Encoder::value()
{
  int pos = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = ticks; 
  }
  return pos;
}
//*******Functions*********



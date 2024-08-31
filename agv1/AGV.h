#ifndef AGV_H
#define AGV_H
#include <Arduino.h>
//#include <QTRSensors.h>

//********Driver Class************
class Driver 
{
  private:
    byte R_EN;
    byte L_EN;
    byte RPWM;
    byte LPWM;

    byte direction;
  public:
    Driver() {} // Do not use 
    Driver(byte R_EN, byte L_EN, byte RPWM, byte LPWM);

    void init();
    byte forward(int speed);
    byte backward(int speed);

    void up();
    void down();

    void stop();
};

//*********Robot Class************
class Robot
{
  private:
    Driver right_motor;
    Driver left_motor;

    Driver actuator1;
    Driver actuator2;

    int R = 10;
    int L = 50;
  public:
    Robot() {} // Do not use
    Robot(Driver &right_motor, Driver &left_motor);
    Robot(Driver &right_motor, Driver &left_motor, Driver actuator1, Driver actuator2);

    void init();
    #define FORWARD 1
    #define BACKWARD 0
    void start_move(byte direction, int speed);
    void forward(int right_speed, int left_speed);
    void backward(int right_speed, int left_speed);

    void right(int right_speed, int left_speed);
    void left(int right_speed, int left_speed);

    void up();
    void down();

    void stop_lift();
    void stop_move();
};
//************Encoder Class************
class Encoder 
{
  private:
    byte ENA;
    byte ENB;

    volatile int ticks = 0;

    static Encoder* sEncoder ;
    static void updateEncoderISR();

    void pulse();
    
  public:
    Encoder(){} // Do not use 
    Encoder(byte ENA, byte ENB);
    
    void init();
    int value();
};
//*******Functions**********


#endif
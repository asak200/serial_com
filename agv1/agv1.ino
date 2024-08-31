#include "AGV.h"
// #include <QTRSensors.h>
// #include <NewPing.h>

//           R_EN, L_EN, RPWM, LPWM
Driver left_motor(23, 25, 4, 5);
Driver right_motor(31, 33, 8, 9);
//           R_EN, L_EN, RPWM, LPWM
Driver actuator1(27, 29, 6, 7);
Driver actuator2(35, 37, 10, 11);

Robot robot(right_motor, left_motor, actuator1, actuator2);
// Encoder encoder_l(18,19);
// Encoder encoder_r(20,21);

// QTRSensors qtr;
// const uint8_t SensorCount = 8; // IR Sensors Count 
// uint16_t sensorValues[SensorCount];

#define TRIGGER_PIN 11
#define ECHO_PIN 12
// NewPing sonar(TRIGGER_PIN,ECHO_PIN);

#define Kp 0.025
#define Kd 0.0001
#define MaxSpeed 180
#define BaseSpeed 150

// ASAK'S VARIABLES (DON'T TOUCH)
#define maxSpeed 0.54
#define minSpeed 0.2
#define maxPWM 255
#define minPWM 100

#define WHEEL_RAD 0.1
#define WHEEL_SAP 0.55
#define ENC_COUNT_PER_REV 2400

float distance_per_count = 2*3.14159 * WHEEL_RAD / ENC_COUNT_PER_REV;

// encoders
int encoder_Pin_1 = 18;
int encoder_Pin_2 = 19;
int encoder_Pin_3 = 20;
int encoder_Pin_4 = 21;
long prv_el, prv_er;
volatile long er, el;
volatile long lastEncoded, lastEncoded1;

// motor speed reqeuests
double pt, t, dt;
float xtl, xtr, rev, vl, vr, svl, svr, xr, xl, ang, st_ang;
String str_sr = "0.00", str_sl = "0.00", psl, psr;
float sr, sl, read_sl, read_sr, prsl, prsr;
short v_c, qr_order, sl_0, sr_0;

float leftPIDOutput, rightPIDOutput;
int leftPWM, rightPWM;

// PID constants
float kp = 1.;
float ki = .5;
float kd = 0.01;

// PID structure
struct PID {
  float kp, ki, kd;
  float error, integral, derivative;
  float previousError;
};
PID leftPID, rightPID;

float calculatePID(PID &pid, float setpoint, float currentRead, float dtt) {
  pid.error = setpoint - currentRead;
  pid.integral += pid.error * dtt;
  pid.derivative = (pid.error - pid.previousError) / dtt;
  pid.previousError = pid.error;

  float output = pid.kp * pid.error + pid.ki * pid.integral + pid.kd * pid.derivative;
  if (setpoint == 0.0) output = 0;
  return constrain(output, -maxSpeed, maxSpeed);
}
// Function to calculate PWM from PID output
int calculatePWM(float pidOutput) {
  int pwm = 0;
  if (pidOutput > 0.05) {
    pwm = (pidOutput) * (maxPWM - minPWM) / (maxSpeed) + minPWM;
    pwm = constrain(pwm, minPWM, maxPWM);
  }
  else if (pidOutput < -0.05){
    pwm = (-pidOutput) * (maxPWM - minPWM) / (maxSpeed) + minPWM;
    pwm = constrain(pwm, minPWM, maxPWM);
  }
  return pwm;
}

void start_robot(Robot*);
void stop_robot(Robot*);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  robot.init();

  digitalWrite(encoder_Pin_1, HIGH); //turn pullup resistor on
  digitalWrite(encoder_Pin_2, HIGH); //turn pullup resistor on

  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  attachInterrupt(digitalPinToInterrupt(18), updateEncoder, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(19), updateEncoder, CHANGE);
  //second
  pinMode(encoder_Pin_3, INPUT); 
  pinMode(encoder_Pin_4, INPUT);

  digitalWrite(encoder_Pin_3, HIGH); //turn pullup resistor on
  digitalWrite(encoder_Pin_4, HIGH); //turn pullup resistor on

  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  attachInterrupt(digitalPinToInterrupt(20), updateEncoder1, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(21), updateEncoder1, CHANGE);

  // init PIDs
  leftPID.kp = kp;
  leftPID.ki = ki;
  leftPID.kd = kd;
  rightPID.kp = kp;
  rightPID.ki = ki;
  rightPID.kd = kd;

}

void loop() {
  if (Serial.available()){
    String msg = Serial.readStringUntil('\n'); // to recieve
    if (msg[0] == 'v' && msg[1] == 's' && qr_order == 0) get_vel(msg);
    else if (msg[0] == 'o' && msg[1] == 'r') get_orded(msg);
  }
  get_real_vel_pose();
  turn_90_degs();

  // robot.forward(200, 200);
  
  leftPIDOutput = calculatePID(leftPID, sl, vl, dt);
  rightPIDOutput = calculatePID(rightPID, sr, vr, dt);
  
  leftPWM = calculatePWM(leftPIDOutput);
  rightPWM = calculatePWM(rightPIDOutput);

  if (sl == 0 && sr == 0) robot.stop_move();
  else if (sl >= 0 && sr >= 0) robot.forward(rightPWM, leftPWM);
  else if (sl <= 0 && sr <= 0) robot.backward(rightPWM, leftPWM);
  else if (sl < 0 && sr > 0) robot.left(rightPWM, leftPWM);
  else if (sl > 0 && sr < 0) robot.right(rightPWM, leftPWM);

  delay(10);
}

// vs: 0.55 0.30
void get_vel(String a){
  str_sl[2] = a[6]; str_sl[3] = a[7];
  read_sl = str_sl.toFloat();
  if (read_sl > 0.6) read_sl = 0.6;
  if (a[3] == '-') read_sl *= -1;
  
  str_sr[2] = a[11]; str_sr[3] = a[12];
  read_sr = str_sr.toFloat();
  if (read_sr > 0.6) read_sr = 0.6;
  if (a[8] == '-') read_sr *= -1;

  if (read_sl == 0) sl_0++;
  else {
    sl = read_sl;
    sl_0 = 0;  
  }
  if (sl_0 > 2){
    sl = read_sl;
    sl_0 = 0;
  }


  if (read_sr == 0) sr_0++;
  else {
    sr = read_sr;
    sr_0 = 0;  
  }
  if (sr_0 > 2){
    sr = read_sr;
    sr_0 = 0;
  }
  prsl = sl;
  prsr = sr;
}
void get_orded(String a){
  char qr = a[4];
  if (qr == 'l') {
    qr_order = 1;
    st_ang = ang;
  }
  else if (qr == 'r') {
    qr_order = -1;
    st_ang = ang;
  }

  while (Serial.available()) Serial.read();
}
void get_real_vel_pose(){
  // x = 2t (m)
  // rev = x /(2*pi*r) (rad)
  // enc = 2500 * rev
  // xtl = 0.6*t;
  // xtr = 0.3*t;
  
  // el = xtl /distance_per_count;
  // er = xtr /distance_per_count;

  // calculate dx and measure the displacement
  int del = el - prv_el;
  float dxl = del * distance_per_count;
  prv_el = el;
  xl += dxl;

  int der = er - prv_er;
  float dxr = der * distance_per_count;
  prv_er = er;
  xr += dxr;

  // calculate angle
  float angl = (dxr-dxl)/WHEEL_SAP * 180 / 3.14159;
  ang += angl;
  
  // get dt and calculate the velocity
  t = millis() / 1000.;
  dt = t - pt;
  pt = t;

  if (dt>0) {
    svl += dxl/dt;
    svr += dxr/dt;
    v_c++;
  }
  // get the mean every 5 velocity calculations
  if (v_c >= 5) {
    vl = svl/5; vr = svr/5; 
    svl=0; svr=0; 
    v_c=0;
  }
  
  Serial.print("enc: ");
    Serial.print(xl, 5);
    Serial.print("  ");
    Serial.print(xr, 5);
    Serial.print("  ");
    Serial.print(vl);
    Serial.print("  ");
    Serial.print(vr);
    Serial.print("  ");
    Serial.print(sl);
    Serial.print("  ");
    Serial.print(sr);
    Serial.print("  ");
    Serial.print(ang, 5);
    Serial.print("  ");
    Serial.print(qr_order);
    
    Serial.println("");
}
void turn_90_degs(){
  if(qr_order == 1 && (ang - st_ang) < 90.){
    sl = 0.;
    sr = 0.5;
  }
  else if(qr_order == -1 && (ang - st_ang) > -90.){
    sl = 0.5;
    sr = 0.;
  }
  else{
    qr_order = 0;
    sl = prsl;
    sr = prsr;
  }
}

void updateEncoder(){
  int MSB = digitalRead(encoder_Pin_1); //MSB = most significant bit
  int LSB = digitalRead(encoder_Pin_2); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) el --;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) el ++;

  lastEncoded = encoded; //store this value for next time
}

void updateEncoder1(){
  int MSB1 = digitalRead(encoder_Pin_3); //MSB = most significant bit
  int LSB1 = digitalRead(encoder_Pin_4); //LSB = least significant bit

  int encoded1 = (MSB1 << 1) |LSB1; //converting the 2 pin value to single number
  int sum  = (lastEncoded1 << 2) | encoded1; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) er ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) er --;

  lastEncoded1 = encoded1; //store this value for next time
}

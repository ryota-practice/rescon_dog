#include <math.h>
#include <Wire.h>
#include <string.h>
#include <PCA9685.h>

#define SERVOMIN 150  //最小パルス幅 (標準的なサーボパルスに設定)
#define SERVOMAX 600  //最大パルス幅 (標準的なサーボパルスに設定)

#define SERVONUMBER 12

#define FR 0  //leg position->number
#define FL 1
#define RR 2
#define RL 3

#define white 0  //dog color
#define black 1

void servo_write(int, int);
void servo_all(int*, int*, bool*);
void pos2ang(int*, int, double, double, double);
void servo_trotmove(int*, int*, bool*, int, int,
                    double, double, double, double, double, double,
                    double, double, double, double, double, double);

const int dogcolor = black;  //set dog color

double L0 = 3;   //length of link
double L1 = 52;  //length of link
double L2 = 52;  //length of link
double x, y, z;  //target position

//{FR2,FL2,RR2,RL2,FR1,FL1,RR1,RL1,FR0,FL0,RR0,RL0}
//{  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11}
const int offset1[12] = { 3, 2, 8, 7, 10, 11, 1, 11, -11, 9, 10, -4 };   //offset of each servo white
const int offset2[12] = { 3, 4, 7, 11, -2, 8, 5, -4, -25, 25, 20, 0 };  //offset of each servo black
const bool servosw[12] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };         //switch
const bool ALL1[12] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };            //all ON

const int angmin[12] = { 20, 20, 20, 20, 30, 30, 30, 30, 20, 20, 20, 20 };              //minimum angle for mg92b
const int angmax[12] = { 160, 160, 160, 160, 160, 160, 160, 160, 160, 160, 160, 160 };  //maximum angle for mg92b
const int ang90[12] = { 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90 };               //90 deg
// const int ang80[12] = { 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80 };               //80deg
// const int ang100[12] = { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100 };  //100deg

int ang[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };     //for write
int offset[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  //for write

PCA9685 pwm = PCA9685(0x40);  //address 0x40(default)

void setup() {
  Serial.begin(9600);  //serial monitor
  pwm.begin();
  pwm.setPWMFreq(60);  //I2C master

  if (dogcolor)                                //set offset of each color
    memcpy(offset, offset2, sizeof(offset2));  //offset black
  else
    memcpy(offset, offset1, sizeof(offset1));  //offset white

  Serial.print("activate wanko...");

  // pos2ang(ang, FR, 0, 90.0666419935816, 3);
  // pos2ang(ang, FL, 0, 90.0666419935816, 3);
  // pos2ang(ang, RR, 0, 90.0666419935816, 3);
  // pos2ang(ang, RL, 0, 90.0666419935816, 3);
  // servo_all(ang, offset, servosw);
  delay(1000);
}

void loop() {
  Serial.println("loop start");
  servo_all(ang90, offset, servosw);
  delay(1000);
  Serial.println("loop end");
}

//custum function
void servo_write(int ch, int ang) {
  ang = map(ang, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(ch, 0, ang);
}

void servo_all(int* input, int* ofs, bool* sw) {
  for (int i = 0; i <= SERVONUMBER - 1; i++) {
    if (sw[i] == 1)
      servo_write(i, input[i] - ofs[i]);
  }
}

void pos2ang(int* input, int legpos, double x, double y, double z) {
  double u;                          //coordinates convert
  double theta_a, theta_b;           //angle for calculate
  double theta_0, theta_1, theta_2;  //objective angle

  theta_0 = atan2(z, y) - asin(L0 / sqrt(square(y) + square(z)));
  u = sqrt(square(y + L0 * sin(theta_0)) + square(z - L0 * cos(theta_0)));
  theta_a = acos((square(x) + square(u) + square(L1) - square(L2)) / (2 * L1 * sqrt(square(x) + square(u))));
  theta_b = acos((square(x) + square(u) - square(L1) + square(L2)) / (2 * L2 * sqrt(square(x) + square(u))));

  //{FR2,FL2,RR2,RL2,FR1,FL1,RR1,RL1,FR0,FL0,RR0,RL0}
  //{  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11}
  switch (legpos) {
    case FR:                                                         //front right
      input[8] = 90 + (int)round(degrees(theta_0)) * 2;              //theta_0
      input[4] = (int)round(degrees(M_PI - theta_a - atan2(u, x)));  //theta_1
      input[0] = (int)round(degrees(M_PI - theta_a - theta_b));      //theta_2
      break;

    case FL:                                                  //front left
      input[9] = 90 - (int)round(degrees(theta_0)) * 2;       //theta_0
      input[5] = (int)round(degrees(theta_a + atan2(u, x)));  //theta_1
      input[1] = (int)round(degrees(theta_a + theta_b));      //theta_2
      break;

    case RR:                                                         //rear right
      input[10] = 90 + (int)round(degrees(theta_0)) * 2;             //theta_0
      input[6] = (int)round(degrees(M_PI - theta_a - atan2(u, x)));  //theta_1
      input[2] = (int)round(degrees(M_PI - theta_a - theta_b));      //theta_2
      break;

    case RL:                                                  //rear left
      input[11] = 90 - (int)round(degrees(theta_0)) * 2;      //theta_0
      input[7] = (int)round(degrees(theta_a + atan2(u, x)));  //theta_1
      input[3] = (int)round(degrees(theta_a + theta_b));      //theta_2
      break;

    default:
      Serial.print("invalid leg position");
  }
}

void servo_trotmove(int* ang, int* offset, bool* servosw, int resolution, int time,                        //setting
                    double FR_x1, double FR_y1, double FR_z1, double FL_x1, double FL_y1, double FL_z1,    //start
                    double FR_x2, double FR_y2, double FR_z2, double FL_x2, double FL_y2, double FL_z2) {  //goal
  double FR_step_x = (FR_x2 - FR_x1) / resolution;
  double FR_step_y = (FR_y2 - FR_y1) / resolution;
  double FR_step_z = (FR_z2 - FR_z1) / resolution;
  double FL_step_x = (FL_x2 - FL_x1) / resolution;
  double FL_step_y = (FL_y2 - FL_y1) / resolution;
  double FL_step_z = (FL_z2 - FL_z1) / resolution;

  for (int i = 0; i <= resolution - 1; i++) {
    Serial.println(i);
    pos2ang(ang, FR, FR_x1 + FR_step_x * i, FR_y1 + FR_step_y * i, FR_z1 + FR_step_z * i);
    pos2ang(ang, FL, FL_x1 + FL_step_x * i, FL_y1 + FL_step_y * i, FL_z1 + FL_step_z * i);
    pos2ang(ang, RR, FL_x1 + FL_step_x * i, FL_y1 + FL_step_y * i, FL_z1 + FL_step_z * i);
    pos2ang(ang, RL, FR_x1 + FR_step_x * i, FR_y1 + FR_step_y * i, FR_z1 + FR_step_z * i);
    servo_all(ang, offset, servosw);
    delay(time);
  }
}
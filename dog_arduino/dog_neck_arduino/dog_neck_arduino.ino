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
void servo_crossmove(int*, int*, bool*, int, int,
                     double, double, double, double, double, double,
                     double, double, double, double, double, double);
void servo_neck(int, int, int, int);

PCA9685 pwm = PCA9685(0x40);  //address 0x40(default)

double L0 = 3;   //length of link
double L1 = 52;  //length of link
double L2 = 52;  //length of link

//{FR2,FL2,RR2,RL2,FR1,FL1,RR1,RL1,FR0,FL0,RR0,RL0}
//{  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11}
const int offset1[12] = { 3, 2, 8, 7, 10, 11, 1, 11, -11, 9, 10, -4 };  //offset of each servo white
const int offset2[12] = { 3, 7, 7, 11, -2, 8, 5, -4, -25, 25, 20, 0 };  //offset of each servo black

const bool servosw[12] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };  //switch (0->OFF 1->ON)

// const int angmin[12] = { 20, 20, 20, 20, 30, 30, 30, 30, 20, 20, 20, 20 };              //minimum angle for mg92b
// const int angmax[12] = { 160, 160, 160, 160, 160, 160, 160, 160, 160, 160, 160, 160 };  //maximum angle for mg92b
const int ang90[12] = { 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90 };  //90 deg

int ang[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };     //for write angle
int offset[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  //for write offset

const int dogcolor = black;  //set dog color

void setup() {
  Serial.begin(9600);  //begin serial
  pwm.begin();         //begin pwm by PCA9685
  pwm.setPWMFreq(60);  //begin I2C master

  //set offset of each color
  switch (dogcolor) {
    case white:
      memcpy(offset, offset1, sizeof(offset1));
      break;
    case black:
      memcpy(offset, offset2, sizeof(offset2));
      break;
    default:
      Serial.println("there is no wanko.");
  }

  Serial.print("activate wanko...");
  servo_all(ang90, offset, servosw);
  Serial.println("done.");
}

void loop() {
  int v_ch = 14;     //vartical ch
  int h_ch = 15;     //horizontal ch
  int v_range = 50;  //vartical range
  int h_range = 50;  //horizontal range
  int v_min = 90 - (int)(v_range / 2);
  int v_max = 90 + (int)(v_range / 2);
  int h_min = 90 - (int)(h_range / 2);
  int h_max = 90 + (int)(h_range / 2);
  bool inv = 1;

  servo_neck(v_ch, h_ch, v_min, h_min);
  delay(250);

  for (int i = v_min; i <= v_max; i += 5) {
    if (inv) {
      for (int j = h_min; j <= h_max; j += 5) {
        servo_neck(v_ch, h_ch, i, j);
        delay(125);
      }
      inv = 0;
    } else {
      for (int j = h_max; j >= h_min; j -= 5) {
        servo_neck(v_ch, h_ch, i, j);
        delay(125);
      }
      inv = 1;
    }
  }
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

void servo_crossmove(int* ang, int* offset, bool* servosw, int resolution, int time,                        //setting
                     double FR_x1, double FR_y1, double FR_z1, double FL_x1, double FL_y1, double FL_z1,    //start
                     double FR_x2, double FR_y2, double FR_z2, double FL_x2, double FL_y2, double FL_z2) {  //goal
  double FR_step_x = (FR_x2 - FR_x1) / resolution;
  double FR_step_y = (FR_y2 - FR_y1) / resolution;
  double FR_step_z = (FR_z2 - FR_z1) / resolution;
  double FL_step_x = (FL_x2 - FL_x1) / resolution;
  double FL_step_y = (FL_y2 - FL_y1) / resolution;
  double FL_step_z = (FL_z2 - FL_z1) / resolution;

  for (int i = 0; i <= resolution - 1; i++) {
    // Serial.println(i);
    pos2ang(ang, FR, FR_x1 + FR_step_x * i, FR_y1 + FR_step_y * i, FR_z1 + FR_step_z * i);
    pos2ang(ang, FL, FL_x1 + FL_step_x * i, FL_y1 + FL_step_y * i, FL_z1 + FL_step_z * i);
    pos2ang(ang, RR, FL_x1 + FL_step_x * i, FL_y1 + FL_step_y * i, FL_z1 + FL_step_z * i);
    pos2ang(ang, RL, FR_x1 + FR_step_x * i, FR_y1 + FR_step_y * i, FR_z1 + FR_step_z * i);
    servo_all(ang, offset, servosw);
    delay(time);
  }
}

void servo_neck(int v_ch, int h_ch, int v, int h) {
  servo_write(v_ch, v);
  servo_write(h_ch, h);
}
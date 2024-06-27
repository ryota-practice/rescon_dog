#include <math.h>
#include <Wire.h>
#include <string.h>
#include <Adafruit_PWMServoDriver.h>

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
double square(double);
void servo_neck(int, int, int, int);

// PCA9685サーボモータードライバのオブジェクトを作成
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

double L0 = 3;   //length of link
double L1 = 52;  //length of link
double L2 = 52;  //length of link

//{FR2,FL2,RR2,RL2,FR1,FL1,RR1,RL1,FR0,FL0,RR0,RL0}
//{  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11}
int offset1[12] = { 7, 2, 6, 7, 4, 11, -5, 20, -26, 36, 40, -15 };  //offset of each servo white
int offset2[12] = { 3, 7, -7, 3, -2, 8, -2, -8, -25, 25, 20, 10 };  //offset of each servo black

bool servosw[12] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };  //switch (0->OFF 1->ON)

// const int angmin[12] = { 20, 20, 20, 20, 30, 30, 30, 30, 20, 20, 20, 20 };              //minimum angle for mg92b
// const int angmax[12] = { 160, 160, 160, 160, 160, 160, 160, 160, 160, 160, 160, 160 };  //maximum angle for mg92b
int ang90[12] = { 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90 };  //90 deg

int ang[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };     //for write angle
int offset[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  //for write offset

//neck parameter
int v_center;
int h_center;

int v_ch = 14;     //vartical ch
int h_ch = 15;     //horizontal ch
int v_range = 40;  //vartical range
int h_range = 50;  //horizontal range
int v_min = v_center - (int)(v_range / 2);
int v_max = v_center + (int)(v_range / 2);
int h_min = h_center - (int)(h_range / 2);
int h_max = h_center + (int)(h_range / 2);

int neck_d = 1500;

const int dogcolor = white;  //set dog color

void setup() {
  Serial.begin(115200);  //begin serial
  pwm.begin();           //initialize and begin pwm by PCA9685
  pwm.setOscillatorFrequency(27000000);
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
  delay(1000);
  servo_crossmove(ang, offset, servosw, 10, 50,
                  25, 52, 3, 25, 52, 3,
                  0, 60, 3, 0, 60, 3);
  servo_crossmove(ang, offset, servosw, 5, 5,
                  0, 60, 3, 0, 60, 3,
                  -15, 60, 3, -15, 60, 3);
  delay(500);
  servo_crossmove(ang, offset, servosw, 5, 10,
                  0, 52, 3, 0, 52, 3,
                  0, 90.0666419935816, 3, 0, 90.0666419935816, 3);
  delay(1000);
  Serial.println("done.");

  //neck setup
  switch (dogcolor) {
    case white:
      v_center = 95;
      h_center = 75;
      break;
    case black:
      v_center = 95;
      h_center = 75;
      break;
    default:
      Serial.println("there is no wanko.");
  }
}

void loop() {
  Serial.println("loop start");

  int res1 = 20;
  int res2 = 5;
  int d = 5;
  double forward_d = 30;   //forward distance
  double backward_d = 40;  //backward distance
  double waki = 5;
  for (int j = 1; j <= 3; j++) {
    //forward
    for (int i = 1; i <= 6; i++) {
      //1->2
      servo_crossmove(ang, offset, servosw, res1, d,
                      forward_d, 90.0666419935816, waki, 0, 90.0666419935816, waki,
                      0, 90.0666419935816, waki, -backward_d, 90.0666419935816, waki);
      //2->2.5
      servo_crossmove(ang, offset, servosw, res2, d,
                      0, 90.0666419935816, waki, -backward_d, 90.0666419935816, waki,
                      0, 90.0666419935816, waki, 0, 65, waki);
      //2.5->3
      servo_crossmove(ang, offset, servosw, res2, d,
                      0, 90.0666419935816, waki, 0, 65, waki,
                      0, 90.0666419935816, waki, forward_d, 90.0666419935816, waki);
      //3->4
      servo_crossmove(ang, offset, servosw, res1, d,
                      0, 90.0666419935816, waki, forward_d, 90.0666419935816, waki,
                      -backward_d, 90.0666419935816, waki, 0, 90.0666419935816, waki);
      //4->4.5
      servo_crossmove(ang, offset, servosw, res2, d,
                      -backward_d, 90.0666419935816, waki, 0, 90.0666419935816, waki,
                      0, 65, 5, 0, 90.0666419935816, waki);
      //4.5->5=1
      servo_crossmove(ang, offset, servosw, res2, d,
                      0, 65, waki, 0, 90.0666419935816, waki,
                      forward_d, 90.0666419935816, waki, 0, 90.0666419935816, waki);
    }
    servo_crossmove(ang, offset, servosw, res2, d,
                    0, 90.0666419935816, waki, 0, 90.0666419935816, waki,
                    0, 90.0666419935816, waki, 0, 90.0666419935816, waki);

    servo_neck(v_ch, h_ch, v_center, h_min);
    delay(neck_d);
    for (int i = 1; i <= 3; i++) {
      servo_neck(v_ch, h_ch, v_center, h_min + h_range / 4 * i);
      delay(neck_d);
    }
    servo_neck(v_ch, h_ch, v_center, h_max);
    delay(neck_d);
    for (int i = 3; i >= 1; i--) {
      servo_neck(v_ch, h_ch, v_center, h_min + h_range / 4 * i);
      delay(neck_d);
    }
  }

  //歩き終わり 座ってきょろきょろ
  servo_all(ang90, offset, servosw);
  delay(1000);

  while (1) {
    servo_neck(v_ch, h_ch, v_center, h_min);
    delay(neck_d);
    for (int i = 1; i <= 3; i++) {
      servo_neck(v_ch, h_ch, v_center, h_min + h_range / 4 * i);
      delay(neck_d);
    }
    servo_neck(v_ch, h_ch, v_center, h_max);
    delay(neck_d);
    for (int i = 3; i >= 1; i--) {
      servo_neck(v_ch, h_ch, v_center, h_min + h_range / 4 * i);
      delay(neck_d);
    }
  }

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

double square(double n) {
  return n * n;
}

void servo_neck(int v_ch, int h_ch, int v, int h) {
  servo_write(v_ch, v);
  servo_write(h_ch, h);
}
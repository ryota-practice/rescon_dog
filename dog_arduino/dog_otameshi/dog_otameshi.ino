#include <Wire.h>
#include <PCA9685.h>  //PCA9685用ヘッダーファイル（秋月電子通商作成）

PCA9685 pwm = PCA9685(0x41);  //PCA9685のアドレス指定（アドレスジャンパ未接続時）

#define SERVOMIN 150  //最小パルス幅 (標準的なサーボパルスに設定)
#define SERVOMAX 600  //最大パルス幅 (標準的なサーボパルスに設定)

void servo_write(int, int);
void servo_all(int*);  //すべてのサーボへ配列の値を書き込み

//-----------------------------------------------------------------------------

//ch0=FR2,ch1=FL2,ch2=RR2,ch3=RL2,ch4=FR1,ch5=FL1,ch6=RR1,ch7=RL1,ch8=FR0,ch9=FL0,ch10=RR0,ch11=RL0
int i_ang[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };                         //書き込み用配列初期化
const int s_ang[12] = { 154, 30, 148, 28, 56, 110, 75, 103, 125, 98, 82, 90 };  //直立
const int o_ang[12] = { 96, 82, 92, 85, 77, 90, 90, 90, 110, 95, 90, 75 };      //お座り
const int t_ang[12] = { 0, 30, 148, 28, 56, 110, 75, 103, 125, 98, 82, 90 };    //サーボテスト用

void setup() {
  pwm.begin();         //初期設定 (アドレス0x40用)
  pwm.setPWMFreq(60);  //PWM周期を60Hzに設定 (アドレス0x40用)
}

void loop() {
  write_all(s_ang);

  delay(1000);
}

//-----------------------------------------------------------------------------

void servo_write(int ch, int ang) {            //動かすサーボチャンネルと角度を指定
  ang = map(ang, 0, 180, SERVOMIN, SERVOMAX);  //角度（0～180）をPWMのパルス幅（150～600）に変換
  pwm.setPWM(ch, 0, ang);
  //delay(1);
}

void write_all(int* temp) {
  for (int i = 0; i <= 11; i++) {
    servo_write(i, temp[i]);
  }
}
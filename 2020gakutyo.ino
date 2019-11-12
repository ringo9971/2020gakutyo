#include <CytronMotorDriver.h>
#include <Servo.h>
#include <HCSR04.h>
#define MAXSUM 3000

CytronMD right_motor(PWM_PWM, 6, 7);
CytronMD left_motor(PWM_PWM, 8, 9);

Servo arm, wrist;
Servo rfinger, lfinger;

// echo, trig
HCSR04 frontUltrasound(50, 52);

enum Point {highest, lower, halfway, open, close, shoot, maxpoint};
int armpoint[maxpoint];
int wristpoint[maxpoint];
int rfingerpoint[maxpoint];
int lfingerpoint[maxpoint];

const double radius = 5.8/2;
const double tread = 18.0;
const int cnt_par_round = 918;

const double pgain = 1.0, igain = 0.010, dgain = 0;
const double sumpgain = 1.0, sumigain = 0.01;

int16_t rightspeed, leftspeed;
int16_t pastrightspeed = 0, pastleftspeed = 0;
int16_t deviation;

int32_t gap, pastgap = 0;
int32_t gap_sum = 0;
int32_t rightsum = 0, leftsum = 0;
int32_t right_target_cnt = 0, left_target_cnt = 0;
int32_t past_right_target_cnt = 0, past_left_target_cnt = 0;

int32_t delayTimer;
int32_t loopTimer;
int32_t now;
boolean flag = false;
boolean is_first_time = true;

int32_t rightcnt = 0, leftcnt = 0;

void setup(){
  Serial.begin(115200);
  
  pinMode(3, INPUT);
  pinMode(5, INPUT);

  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(2), read_enca, CHANGE);
  attachInterrupt(digitalPinToInterrupt(4), read_encb, CHANGE);

  arm.attach(22);
  wrist.attach(24);
  rfinger.attach(26);
  lfinger.attach(28);

  armpoint[highest] = 730;
  armpoint[lower]   = 2150;
  wristpoint[highest] = 800;
  wristpoint[halfway] = 2300;
  wristpoint[lower]   = 2000;
  wristpoint[shoot] = 600;

  rfingerpoint[open] = 1100;
  rfingerpoint[close] = 700;
  rfingerpoint[shoot] = 800;
  lfingerpoint[open] = 1850;
  lfingerpoint[close] = 2250;
  lfingerpoint[shoot] = 2150;

  arm.writeMicroseconds(1900);
  wrist.writeMicroseconds(1800);
  delay(500);
  rfinger.writeMicroseconds(2250);
  lfinger.writeMicroseconds(650);
  delay(1000);

  delayTimer = millis();
  loopTimer = micros();
}

void loop(){
}


/////////////////////////////////////////////////////////////////
// motor

// 進ませる
void motorDrive(){
  right_motor.setSpeed(rightspeed);
  left_motor.setSpeed(leftspeed);
}

// 目標まで進む
void moveToGoal(int speed, int r, int l){
  rightsum = constrain(rightsum+right_target_cnt-rightcnt, -MAXSUM, MAXSUM);
  leftsum = constrain(leftsum+left_target_cnt-leftcnt, -MAXSUM, MAXSUM);

  // PID
  rightspeed = constrain((right_target_cnt-rightcnt)*pgain+rightsum*igain+(rightspeed-pastrightspeed)*dgain, -speed, speed);
  leftspeed = constrain((left_target_cnt-leftcnt)*pgain+leftsum*igain+(leftspeed-pastleftspeed)*dgain, -speed, speed);

  embed_difference(r, l);

  if(rightcnt == right_target_cnt) rightspeed = 0;
  if(leftcnt == left_target_cnt) leftspeed = 0;

  pastrightspeed = rightspeed;
  pastleftspeed = leftspeed;

  motorDrive();

  do{
    now = micros();
  }while(now-loopTimer <= 1000);
}

// 差を埋める
void embed_difference(int r, int l){
  gap = l*(leftcnt-past_left_target_cnt)-r*(rightcnt-past_right_target_cnt);
  gap_sum = constrain(gap_sum+gap, -MAXSUM, MAXSUM);
  deviation = gap*sumpgain+gap_sum*sumigain;

  rightspeed = constrain(rightspeed+r*deviation, -255, 255);
  leftspeed = constrain(leftspeed-l*deviation, -255, 255);
}

// ターゲットの更新
void updateTargetCnt(){
  past_right_target_cnt = rightcnt;
  past_left_target_cnt = leftcnt;
}

// 目標と一致したか
boolean is_matched(){
  if(rightcnt == right_target_cnt && leftcnt == left_target_cnt){
    rightsum = 0;
    leftsum = 0;
    gap_sum = 0;
    if(delayTimer-millis() >= 500){
      brake();
      updateTargetCnt();
      is_first_time = true;
      return true;
    }
  }else{
    delayTimer = millis();
  }

  return false;
}

// 任意の距離前進
int forward(int speed, double dist){
  loopTimer = micros();
  if(is_first_time){
    right_target_cnt += (double)cnt_par_round/2/PI/radius*dist;
    left_target_cnt += (double)cnt_par_round/2/PI/radius*dist;
    is_first_time = false;
  }

  moveToGoal(speed, 1, 1);

  if(is_matched()) return 0;
  return 1;
}
// 任意の速度で前進
void forward(int speed){
  loopTimer = micros();

  rightspeed = speed;
  leftspeed = speed;
  embed_difference(1, 1);
  motorDrive();

  do{
    now = micros();
  }while(now-loopTimer <= 1000);
}
void forward(){
  forward(255);
}

// 任意の距離後退
int back(int speed, double dist){
  loopTimer = micros();
  if(is_first_time){
    right_target_cnt -= (double)cnt_par_round/2/PI/radius*dist;
    left_target_cnt -= (double)cnt_par_round/2/PI/radius*dist;
    is_first_time = false;
  }

  moveToGoal(speed, -1, -1);

  if(is_matched()) return 0;
  return 1;
}
// 任意の速度で後退
void back(int speed){
  loopTimer = micros();

  rightspeed = -speed;
  leftspeed = -speed;
  embed_difference(-1, -1);
  motorDrive();

  do{
    now = micros();
  }while(now-loopTimer <= 1000);
}
void back(){
  back(255);
}

// 任意の角度右旋回
int right_rotation(int speed, double rad){
  loopTimer = micros();
  if(is_first_time){
    right_target_cnt -= (cnt_par_round*tread)/(720*radius)*rad;
    left_target_cnt += (cnt_par_round*tread)/(720*radius)*rad;
    is_first_time = false;
  }

  moveToGoal(speed, -1, 1);

  if(is_matched()) return 0;
  return 1;
}
// 任意の速度で右旋回
void right_rotation(int speed){
  loopTimer = micros();

  rightspeed = -speed;
  leftspeed = speed;
  embed_difference(-1, 1);
  motorDrive();

  do{
    now = micros();
  }while(now-loopTimer <= 1000);
}
void right_rotation(){
  right_rotation(255);
}

// 任意の角度左旋回
int left_rotation(int speed, double rad){
  loopTimer = micros();
  if(is_first_time){
    right_target_cnt += (cnt_par_round*tread)/(720*radius)*rad;
    left_target_cnt -= (cnt_par_round*tread)/(720*radius)*rad;
    is_first_time = false;
  }

  moveToGoal(speed, 1, -1);

  if(is_matched()) return 0;
  return 1;
}
// 任意の速度で左旋回
void left_rotation(int speed){
  loopTimer = micros();

  rightspeed = speed;
  leftspeed = -speed;
  embed_difference(1, -1);
  motorDrive();

  do{
    now = micros();
  }while(now-loopTimer <= 1000);
}
void left_rotation(){
  left_rotation(255);
}

// ブレーキ
void brake(){
  right_motor.setSpeed(0);
  left_motor.setSpeed(0);
}


/////////////////////////////////////////////////////////////////
// servo
// アームを動かす
void MoveArm(Point e) {
  arm.writeMicroseconds(armpoint[e]);
  wrist.writeMicroseconds(wristpoint[e]);
}

// 指を動かす
void MoveFinger(Point e) {
  rfinger.writeMicroseconds(rfingerpoint[e]);
  lfinger.writeMicroseconds(lfingerpoint[e]);
}

// ボールを持ち上げる動作
void lift() {
  for (int i = wristpoint[lower]; i <= wristpoint[halfway]; i++) {
    wrist.writeMicroseconds(i);
    delay(1);
  }
  delay(1000);
  for (int i = armpoint[lower]; i >= armpoint[highest]; i--) {
    arm.writeMicroseconds(i);
    wrist.writeMicroseconds(int(map(i, armpoint[lower], armpoint[highest], wristpoint[halfway], wristpoint[highest])));
    delay(2);
  }
}

// 上から初期に戻す動作
void take_down() {
  for (int i = armpoint[highest]; i <= armpoint[lower]; i++) {
    arm.writeMicroseconds(i);
    wrist.writeMicroseconds(int(map(i, armpoint[lower], armpoint[highest], wristpoint[lower], wristpoint[shoot])));
    rfinger.writeMicroseconds(int(map(i, armpoint[lower], armpoint[highest], rfingerpoint[open], rfingerpoint[shoot])));
    lfinger.writeMicroseconds(int(map(i, armpoint[lower], armpoint[highest], lfingerpoint[open], lfingerpoint[shoot])));
    delay(2);
  }
}

// シュートする動作
void ballShoot() {
  for(int i = wristpoint[highest]; i >= wristpoint[shoot]; i--){
    wrist.writeMicroseconds(i);
    delay(2);
  }
  rfinger.writeMicroseconds(rfingerpoint[shoot]);
  lfinger.writeMicroseconds(lfingerpoint[shoot]);
}


/////////////////////////////////////////////////////////////////
// 右のエンコーダーを読む
void read_enca(){
  if(!!(PIOB->PIO_PDSR & (1<<25)) == !!(PIOC->PIO_PDSR & (1<<28)))
    rightcnt++;
  else
    rightcnt--;
}

// 左のエンコーダーを読む
void read_encb(){
  if(!!(PIOC->PIO_PDSR & (1<<26)) == !!(PIOC->PIO_PDSR & (1<<25)))
    leftcnt++;
  else
    leftcnt--;
}

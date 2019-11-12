#include <CytronMotorDriver.h>
#define INF 1e7
#define MAXSUM 3000

CytronMD right_motor(PWM_PWM, 6, 7);
CytronMD left_motor(PWM_PWM, 8, 9);

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

  delay(10);

  delayTimer = millis();
  loopTimer = micros();
}

void loop(){
}

void motorDrive(){
  right_motor.setSpeed(rightspeed);
  left_motor.setSpeed(leftspeed);
}

void calc_speed(int speed, int r, int l){
  rightsum = constrain(rightsum+right_target_cnt-rightcnt, -MAXSUM, MAXSUM);
  leftsum = constrain(leftsum+left_target_cnt-leftcnt, -MAXSUM, MAXSUM);

  // PID
  rightspeed = constrain((right_target_cnt-rightcnt)*pgain+rightsum*igain+(rightspeed-pastrightspeed)*dgain, -speed, speed);
  leftspeed = constrain((left_target_cnt-leftcnt)*pgain+leftsum*igain+(leftspeed-pastleftspeed)*dgain, -speed, speed);

  hogehoge(r, l);

  if(rightcnt == right_target_cnt) rightspeed = 0;
  if(leftcnt == left_target_cnt) leftspeed = 0;

  pastrightspeed = rightspeed;
  pastleftspeed = leftspeed;

  // モーターを回す
  motorDrive();

  // 待機児童
  do{
    now = micros();
  }while(now-loopTimer <= 1000);
}

void hogehoge(int r, int l){
  gap = l*(leftcnt-past_left_target_cnt)-r*(rightcnt-past_right_target_cnt);
  gap_sum = constrain(gap_sum+gap, -MAXSUM, MAXSUM);
  deviation = gap*sumpgain+gap_sum*sumigain;

  // 左右の差をなくす
  rightspeed = constrain(rightspeed+r*deviation, -255, 255);
  leftspeed = constrain(leftspeed-l*deviation, -255, 255);
}

void updateTargetCnt(){
  past_right_target_cnt = rightcnt;
  past_left_target_cnt = leftcnt;
}

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

int forward(int speed, double dist){
  loopTimer = micros();
  if(is_first_time){
    right_target_cnt += (double)cnt_par_round/2/PI/radius*dist;
    left_target_cnt += (double)cnt_par_round/2/PI/radius*dist;
    is_first_time = false;
  }

  calc_speed(speed, 1, 1);

  if(is_matched()) return 0;
  return 1;
}
void forward(int speed){
  loopTimer = micros();

  rightspeed = speed;
  leftspeed = speed;
  hogehoge(1, 1);
  motorDrive();

  do{
    now = micros();
  }while(now-loopTimer <= 1000);
}
void forward(){
  forward(255);
}

int back(int speed, double dist){
  loopTimer = micros();
  if(is_first_time){
    right_target_cnt -= (double)cnt_par_round/2/PI/radius*dist;
    left_target_cnt -= (double)cnt_par_round/2/PI/radius*dist;
    is_first_time = false;
  }

  calc_speed(speed, -1, -1);

  if(is_matched()) return 0;
  return 1;
}
void back(int speed){
  loopTimer = micros();

  rightspeed = -speed;
  leftspeed = -speed;
  hogehoge(-1, -1);
  motorDrive();

  do{
    now = micros();
  }while(now-loopTimer <= 1000);
}
void back(){
  back(255);
}

int right_rotation(int speed, double rad){
  loopTimer = micros();
  if(is_first_time){
    right_target_cnt -= (cnt_par_round*tread)/(720*radius)*rad;
    left_target_cnt += (cnt_par_round*tread)/(720*radius)*rad;
    is_first_time = false;
  }

  calc_speed(speed, -1, 1);

  if(is_matched()) return 0;
  return 1;
}
void right_rotation(int speed){
  rightspeed = -speed;
  leftspeed = speed;
  hogehoge(-1, 1);
  motorDrive();
  do{
    now = micros();
  }while(now-loopTimer <= 1000);
}
void right_rotation(){
  right_rotation(255);
}

int left_rotation(int speed, double rad){
  loopTimer = micros();
  if(is_first_time){
    right_target_cnt += (cnt_par_round*tread)/(720*radius)*rad;
    left_target_cnt -= (cnt_par_round*tread)/(720*radius)*rad;
    is_first_time = false;
  }

  calc_speed(speed, 1, -1);

  if(is_matched()) return 0;
  return 1;
}
void left_rotation(int speed){
  loopTimer = micros();

  rightspeed = speed;
  leftspeed = -speed;
  hogehoge(1, -1);
  motorDrive();

  do{
    now = micros();
  }while(now-loopTimer <= 1000);
}
void left_rotation(){
  left_rotation(255);
}

void brake(){
  right_motor.setSpeed(0);
  left_motor.setSpeed(0);
}


void read_enca(){
  if(!!(PIOB->PIO_PDSR & (1<<25)) == !!(PIOC->PIO_PDSR & (1<<28)))
    rightcnt++;
  else
    rightcnt--;
}

void read_encb(){
  if(!!(PIOC->PIO_PDSR & (1<<26)) == !!(PIOC->PIO_PDSR & (1<<25)))
    leftcnt++;
  else
    leftcnt--;
}

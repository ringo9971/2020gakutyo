#include <CytronMotorDriver.h>
#define INF 1e9

CytronMD right_motor(PWM_PWM, 6, 7);
CytronMD left_motor(PWM_PWM, 8, 9);

const int radius = 5.8;
const int tread = 18.0;
const int cnt_par_round = 918;
const double pgain = 1.5, igain = 0.012, dgain = 1;
const double sumpgain = 1.0, sumigain = 0.01;

int gap, pastgap = 0;
int gap_sum = 0;
int manipulation;
int rightsum = 0, leftsum = 0;
int rightspeed, leftspeed;
int pastrightspeed = 0, pastleftspeed = 0;
int right_target_cnt = 0, left_target_cnt = 0;
int past_right_target_cnt = 0, past_left_target_cnt = 0;

int timer;
boolean flag = false;
boolean is_first_time = true;

int rightcnt = 0, leftcnt = 0;

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

  timer = millis();
}

void loop(){
  while(forward(100)){
    Serial.print(rightcnt);
    Serial.print("\t");
    Serial.print(leftcnt);
    Serial.print("\t");

    Serial.print(gap*sumpgain);
    Serial.print("\t");
    Serial.print(gap_sum*sumigain);
    Serial.print("\t");
    Serial.print(manipulation);
    Serial.print("\t");
    Serial.print(rightspeed);
    Serial.print("\n");
  };
  delay(2000);
}

void calc_speed(int speed){
  manipulation = gap*sumpgain+gap_sum*sumigain;
  rightsum = constrain(rightsum+right_target_cnt-rightcnt, -2000, 2000);
  leftsum = constrain(leftsum+left_target_cnt-leftcnt, -2000, 2000);
  rightspeed = constrain((right_target_cnt-rightcnt)*pgain+rightsum*igain, -speed, speed);
  leftspeed = constrain((left_target_cnt-leftcnt)*pgain+leftsum*igain, -speed, speed);

  rightspeed += dgain*(rightspeed-pastrightspeed);
  leftspeed += dgain*(leftspeed-pastleftspeed);

  rightspeed = constrain(rightspeed+manipulation, -speed, speed);
  leftspeed = constrain(leftspeed-manipulation, -speed, speed);

  if(rightcnt == right_target_cnt) rightspeed = 0;
  if(leftcnt == left_target_cnt) leftspeed = 0;
}

boolean is_matched(){
  if(rightcnt == right_target_cnt && leftcnt == left_target_cnt){
    rightsum = 0;
    leftsum = 0;
    gap_sum = 0;
    if(timer-millis() >= 500){
      past_right_target_cnt = right_target_cnt;
      past_left_target_cnt = left_target_cnt;
      brake();
      is_first_time = true;
      return true;
    }
  }else{
    timer = millis();
  }

  return false;
}

int forward(int speed, double dist){
  if(is_first_time){
    right_target_cnt += (double)cnt_par_round/2/PI/radius*dist;
    left_target_cnt += (double)cnt_par_round/2/PI/radius*dist;
    is_first_time = false;
  }

  // 左右の回転数差
  gap = (leftcnt-past_left_target_cnt)-(rightcnt-past_right_target_cnt);
  // 左右の差の合計を出す
  gap_sum = constrain(gap_sum+gap, -2000, 2000);
  calc_speed(speed);

  right_motor.setSpeed(rightspeed);
  left_motor.setSpeed(leftspeed);


  pastrightspeed = rightspeed;
  pastleftspeed = leftspeed;

  if(is_matched()) return 0;
  return 1;
}
int forward(int speed){
  return forward(speed, INF);
}
int forward(){
  return forward(255);
}


int brake(){
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

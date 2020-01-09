#include <CytronMotorDriver.h>
#include <Servo.h>
#include <HCSR04.h>
#define MAXSUM 3000

// motor constructor
CytronMD right_motor(PWM_PWM, 2, 7);
CytronMD left_motor(PWM_PWM, 3, 4);

// servo constructor
Servo arm, wrist;
Servo rfinger, lfinger;

// ultrasound sensor constructor
HCSR04 frontUltrasound(19, 17);
HCSR04 rightUltrasound(14, 15);
HCSR04  leftUltrasound(18, 28);

// robot coordinate
double x = 0, y = 0, angle = 0;
// 0~14
int line = 3;

// robot constant
const double radius = 5.837/2;
const double tread = 18.5;
const int cnt_par_round = 918;

// gain
const double pgain = 1.0, igain = 0.010, dgain = 0;
const double sumpgain = 1.8, sumigain = 0.0003, sumdgain = 0.0;

// motor speed
int16_t rightspeed, leftspeed;
int16_t pastrightspeed = 0, pastleftspeed = 0;
int16_t deviation;

// servo
enum Point {highest, lower, halfway, open, close, shoot, maxpoint};
int armpoint[maxpoint];
int wristpoint[maxpoint];
int rfingerpoint[maxpoint];
int lfingerpoint[maxpoint];

// foto sensor
const int FOT_NUM = 2;
const int MAXLIGHT = 10000;
const int MINLIGHT = 0;
const int MIDDLELIGHT = (MAXLIGHT+MINLIGHT)/2;
int maxBrightness;
int minBrightness;

int32_t light[FOT_NUM];
int32_t pastlight[FOT_NUM];
// 白黒
/* int32_t maxlight[FOT_NUM] = {971, 971}; */
/* int32_t minlight[FOT_NUM] = {  0,   0}; */
// 本番に似た環境
/* int32_t maxlight[FOT_NUM] = {  /1* 316, 277 *1/  2678*0.9, 2771*0.9  }; */
/* int32_t minlight[FOT_NUM] = {  /1* 58, 58 *1/  30*20,  31*20  }; */
// 本番環境
int32_t maxlight[FOT_NUM] = {2729, 2907};
int32_t minlight[FOT_NUM] = {645,   650};

int32_t middlelight[FOT_NUM] = {(maxlight[0]+minlight[0])/2, (maxlight[1]+minlight[1])/2};

// encoder
int32_t rightcnt = 0, leftcnt = 0;
int32_t gap, pastgap = 0;
int32_t gap_sum = 0;
int32_t rightsum = 0, leftsum = 0;
int32_t right_target_cnt = 0, left_target_cnt = 0;

// manipulation
int32_t gap_angle, pastgap_angle = 0;
enum Color {WhiteToBrack, BrackToWhite, White, Brack};
Color pastcolor = BRACK;
Color color = BRACK;

// timer
int32_t motorStartTimer;
int32_t delayTimer;
int32_t loopTimer;
int32_t now;


void setup(){
  Serial.begin(115200);

  // フォトセンサの確認用
  /* fot_init(); */
  /* showfotminmax(); */
  /* while(1); */

  attachInterrupt(digitalPinToInterrupt(50), read_enca, CHANGE);
  attachInterrupt(digitalPinToInterrupt(46), read_encb, CHANGE);

  servo_init();

  pinMode(19, INPUT);
  pinMode(17, OUTPUT);
  pinMode(14, INPUT);
  pinMode(15, OUTPUT);
  pinMode(18, INPUT);
  pinMode(28, OUTPUT);

  // 左を向いてスタート
  angle += 90;
  delayTimer = millis();
}

void loop(){
  color = getColor
  if(getColor() == White) MoveVertically(WhiteToBrack);
  else MoveVertically(BrackToWhite);

  forward(100, 10);

  while(1);
}

Color getColor(){
  readfot();
  if(light[1] <= middlelight[1]) return White;
  return Brack;
}

void updateline(){
  color = getColor();
  if(pastcolor != Color){
    if(90 < angle && angle < 270) line--;
    else line++;
  }
  pastcolor = color;
}

int MoveVertically(Color color){
  int past = 1, ima = 1, sum = 0, change = 0;
  int32_t timer = millis();
  while(true){
    readfot();
    fot_normalize();

    if(maxBrightness <= MIDDLELIGHT){
      if(color == WhiteToBrack) ima = 1;
      if(color == BrackToWhite) ima = 0;
    }else if(minBrightness >= MIDDLELIGHT){
      if(color == WhiteToBrack) ima = 0;
      if(color == BrackToWhite) ima = 1;
    }else{
      ima = 2;
    }

    if(ima == 2 && past != 2) angle_modified();
    else if(ima != past) coordinate_modified();

    if(ima == 1) forward(max(90-change*10, 10));
    if(ima == 0) back(max(90-change*10, 10));
    if(ima == 2){
      
      int omg = map(maxBrightness-minBrightness, 4000, 6000, 4, 50);
      omg = constrain(omg, 4, 50);
      sum = constrain(sum+omg/4, -MAXSUM, MAXSUM);

      if(light[0] < light[1]){
        if(color == WhiteToBrack) right_rotation(omg+sum*0.04);
        if(color == BrackToWhite)  left_rotation(omg+sum*0.04);
      }
      else{
        if(color == WhiteToBrack) left_rotation(omg+sum*0.04);
        if(color == BrackToWhite) right_rotation(omg+sum*0.04);
      }

      // 収束
      if(maxBrightness-minBrightness <= 400){
        brake();
        if(millis()-timer <= 500) break;
      }else{
        timer = millis();
      }
    }

    if(ima != past) change++;

    past = ima;
    wait();
  }
}


void debug(){
  Serial.print(x);
  Serial.print("\t");
  Serial.print(y);
  Serial.print("\t");
  Serial.print(angle);
  Serial.print("\n");
}

/////////////////////////////////////////////////////////////////
// motor
/////////////////////////////////////////////////////////////////
// 任意の座標に移動
void moving_location(int speed, double x1, double y1){
  face_any_angle(speed, atan2(y1-y, x1-x)*180/PI);
  forward(speed, sqrt((y1-y)*(y1-y)+(x1-x)*(x1-x)));
}

// 任意の角度を向く
void face_any_angle(int speed, double x){
  if(x < angle){
    if(angle-x < 180) right_rotation(speed, angle-x);
    else left_rotation(speed, x+360-angle);
  }else{
    if(x-angle < 180) left_rotation(speed, x-angle);
    else right_rotation(speed, angle+360-x);
  }
}

// 角度を合わせる
void angle_modified(){
  brake();
  angle += (rightcnt-right_target_cnt)*720*radius/(cnt_par_round*tread);
  while(angle < 0) angle += 360;
  while(angle > 360) angle -= 360;
  updateTargetCnt();
}

// 座標を合わせる
void coordinate_modified(){
  brake();
  double dist = (rightcnt-right_target_cnt)*2*PI*radius/cnt_par_round;
  x += dist*cos(radius/180*PI);
  y += dist*sin(radius/180*PI);
  updateTargetCnt();
}

// ボールの向きに向く
void fit_to_ball(int speed){
  while(1){
    loopTimer = micros();
    gap_angle = constrain(map(analogRead(0)-analogRead(1), -700, 700, -speed, speed), -speed, speed);

    if(gap_angle == 0) break;
    if(gap_angle*pastgap_angle < 0) break;

    if(gap_angle > 0){
      left_rotation(max(35, abs(gap_angle)));
    }else if(gap_angle < 0){
      right_rotation(max(35, abs(gap_angle)));
    }

    pastgap_angle = gap_angle;
    wait();
  }
  pastgap_angle = 0;
  angle_modified();
}

// 待機
void wait(){
  loopTimer = micros();
  do{
    now = micros();
  }while(now-loopTimer <= 1000);
}

// 進ませる
void motorDrive(){
  rightspeed = constrain(rightspeed, -255, 255);
  leftspeed = constrain(leftspeed, -255, 255);
  right_motor.setSpeed(rightspeed);
  left_motor.setSpeed(leftspeed);
}

// 目標まで進む
void moveToGoal(int speed, int r, int l){
  updateline();

  rightsum = constrain(rightsum+right_target_cnt-rightcnt, -MAXSUM, MAXSUM);
  leftsum  = constrain(leftsum+left_target_cnt-leftcnt, -MAXSUM, MAXSUM);

  // PID
  rightspeed = constrain((right_target_cnt-rightcnt)*pgain+rightsum*igain+(rightspeed-pastrightspeed)*dgain, -speed, speed);
  leftspeed  = constrain((left_target_cnt-leftcnt)*pgain+leftsum*igain+(leftspeed-pastleftspeed)*dgain, -speed, speed);

  double gain = (double)(millis()-motorStartTimer)/1500;
  if(1.0 < gain) gain = 1.0;
  rightspeed *= gain;
  leftspeed *= gain;

  embed_difference(r, l);

  if(rightcnt == right_target_cnt) rightspeed = 0;
  if(leftcnt  == left_target_cnt) leftspeed   = 0;

  pastrightspeed = rightspeed;
  pastleftspeed  = leftspeed;

  motorDrive();

  wait();
}

// 差を埋める
void embed_difference(int r, int l){
  gap = l*(leftcnt-left_target_cnt)-r*(rightcnt-right_target_cnt);
  gap_sum = constrain(gap_sum+gap, -MAXSUM, MAXSUM);
  // PID
  deviation = gap*sumpgain+gap_sum*sumigain+(gap-pastgap)*sumdgain;

  rightspeed = constrain(rightspeed+r*deviation, -255, 255);
  leftspeed  = constrain(leftspeed-l*deviation, -255, 255);

  pastgap = gap;
}

// ターゲットの更新
void updateTargetCnt(){
  right_target_cnt = rightcnt;
  left_target_cnt  = leftcnt;
}

// 目標と一致したか
boolean is_matched(){
  if(rightcnt == right_target_cnt && leftcnt == left_target_cnt){
    rightsum = 0;
    leftsum  = 0;
    gap_sum  = 0;
    if(delayTimer-millis() >= 500){
      brake();
      return true;
    }
  }else{
    delayTimer = millis();
  }

  return false;
}

// 任意の距離前進
void forward(int speed, double dist){
  right_target_cnt += (double)cnt_par_round/2/PI/radius*dist;
  left_target_cnt  += (double)cnt_par_round/2/PI/radius*dist;
  motorStartTimer = millis();
  x += dist*cos(angle/180*PI);
  y += dist*sin(angle/180*PI);

  while(!is_matched()){
    moveToGoal(speed, 1, 1);
  }
}
// 任意の速度で前進
void forward(int speed){
  updateline();

  rightspeed = speed;
  leftspeed  = speed;
  embed_difference(1, 1);
  motorDrive();

  wait();
}
void forward(){
  forward(150);
}

// 任意の距離後退
void back(int speed, double dist){
  right_target_cnt -= (double)cnt_par_round/2/PI/radius*dist;
  left_target_cnt  -= (double)cnt_par_round/2/PI/radius*dist;
  x -= dist*cos(angle/180*PI);
  y -= dist*sin(angle/180*PI);
  motorStartTimer = millis();

  while(!is_matched()){
    moveToGoal(speed, -1, -1);
  }
}
// 任意の速度で後退
void back(int speed){
  updateline();

  rightspeed = -speed;
  leftspeed  = -speed;
  embed_difference(-1, -1);
  motorDrive();

  wait();
}
void back(){
  back(150);
}

// 任意の角度右旋回
void right_rotation(int speed, double rad){
  right_target_cnt -= (cnt_par_round*tread)/(720*radius)*rad;
  left_target_cnt  += (cnt_par_round*tread)/(720*radius)*rad;
  angle -= rad;
  while(angle < 0) angle += 360;
  motorStartTimer = millis();

  while(!is_matched()){
    moveToGoal(speed, -1, 1);
  }
}
// 任意の速度で右旋回
void right_rotation(int speed){
  rightspeed = -speed;
  leftspeed  = speed;
  embed_difference(-1, 1);
  motorDrive();

  wait();
}
void right_rotation(){
  right_rotation(150);
}

// 任意の角度左旋回
void left_rotation(int speed, double rad){
  right_target_cnt += (cnt_par_round*tread)/(720*radius)*rad;
  left_target_cnt  -= (cnt_par_round*tread)/(720*radius)*rad;
  motorStartTimer = millis();
  angle += rad;
  while(angle > 360) angle -= 360;

  while(!is_matched()){
    moveToGoal(speed, 1, -1);
  }
}
// 任意の速度で左旋回
void left_rotation(int speed){
  rightspeed = speed;
  leftspeed  = -speed;
  embed_difference(1, -1);
  motorDrive();

  wait();
}
void left_rotation(){
  left_rotation(150);
}

// ブレーキ
void brake(int time){
  brake();
  delay(time);
}
void brake(){
  right_motor.setSpeed(0);
  left_motor.setSpeed(0);
}


/////////////////////////////////////////////////////////////////
// servo
/////////////////////////////////////////////////////////////////

// サーボ角の初期値ぎめ
void servo_init(){
  arm.attach(21);
  wrist.attach(20);
  rfinger.attach(5);
  lfinger.attach(6);

  armpoint[highest]   = 730;
  armpoint[lower]     = 2100;
  wristpoint[highest] = 800;
  wristpoint[halfway] = 2300;
  wristpoint[lower]   = 2000;
  wristpoint[shoot]   = 600;

  rfingerpoint[open]  = 1100;
  rfingerpoint[close] = 700;
  rfingerpoint[shoot] = 800;
  lfingerpoint[open]  = 1850;
  lfingerpoint[close] = 2250;
  lfingerpoint[shoot] = 2150;

  arm.writeMicroseconds(1900);
  wrist.writeMicroseconds(1800);
  delay(500);
  rfinger.writeMicroseconds(2250);
  lfinger.writeMicroseconds(650);
  delay(1000);
}

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

// ボールを保持する
void ballPreserve(){
  for (int i = wristpoint[lower]; i <= wristpoint[halfway]; i++) {
    wrist.writeMicroseconds(i);
    delay(1);
  }
}

// ボールを持ち上げる動作
void ballLift() {
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
// foto sensor
/////////////////////////////////////////////////////////////////
// センサを読む
void fot_init(){
  int start = millis();
  for(int i = 0; i < FOT_NUM; i++){
    maxlight[i] = -100000;
    minlight[i] = 100000;
  }
  while(millis()-start <= 5000){
    readfot();
    getminmax();
  }
}

void readfot(){
  for(int i = 0; i < FOT_NUM; i++){
    light[i] = 0;
  }

  for(int i = 0; i < 10; i++){
    light[0] += analogRead(2);
    light[1] += analogRead(9);
  }

  pastlight[0] = light[0];
  pastlight[1] = light[1];
}

// 最大値と最小値を更新
void getminmax(){
  for(int i = 0; i < FOT_NUM; i++){
    maxlight[i] = max(maxlight[i], light[i]);
    minlight[i] = min(minlight[i], light[i]);
  }
}

// 正規化
void fot_normalize(){
  for(int i = 0; i < FOT_NUM; i++){
    light[i] = map(light[i], minlight[i], maxlight[i], MINLIGHT, MAXLIGHT);
    light[i] = constrain(light[i], MINLIGHT, MAXLIGHT);

    if(i == 0){
      maxBrightness = light[i];
      minBrightness = light[i];
    }else{
      maxBrightness = max(maxBrightness, light[i]);
      minBrightness = min(minBrightness, light[i]);
    }
  }
}

// debug
void showfot(){
  for(int i = 0; i < FOT_NUM; i++){
    if(i) Serial.print(",");
    Serial.print(light[i]);
  }
  Serial.print("\n");
}
void showfotminmax(){
  Serial.print("MAX: ");
  for(int i = 0; i < FOT_NUM; i++){
    Serial.println(maxlight[i]);
  }
  Serial.print("\n");

  Serial.print("MIN: ");
  for(int i = 0; i < FOT_NUM; i++){
    Serial.println(minlight[i]);
  }
  Serial.print("\n");
}


/////////////////////////////////////////////////////////////////
// encoder
/////////////////////////////////////////////////////////////////
// 右のエンコーダーを読む
void read_enca(){
  if(!!(PIOC->PIO_PDSR & (1<<13)) == !!(PIOC->PIO_PDSR & (1<<15)))
    rightcnt++;
  else
    rightcnt--;
}

// 左のエンコーダーを読む
void read_encb(){
  if(!!(PIOC->PIO_PDSR & (1<<17)) == !!(PIOC->PIO_PDSR & (1<<19)))
    leftcnt++;
  else
    leftcnt--;
}

#include <CytronMotorDriver.h>

CytronMD right_motor(PWM_PWM, 6, 7);
CytronMD left_motor(PWM_PWM, 8, 9);

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
}

void loop(){
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

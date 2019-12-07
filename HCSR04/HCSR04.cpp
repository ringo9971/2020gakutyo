#include "Arduino.h"
#include "HCSR04.h"

HCSR04::HCSR04(int _echopin, int _trigpin){
  echopin = _echopin;
  trigpin = _trigpin;

  cnt = 0;  num = 9;  res = 0;
  for(int i = 0; i < num; i++) dist[i] = 0.0;

  pinMode(echopin, INPUT);
  pinMode(trigpin, OUTPUT);
}

double HCSR04::getdist(double tempera){
  digitalWrite(trigpin, LOW);
  delayMicroseconds(1);
  digitalWrite(trigpin, HIGH);
  delayMicroseconds(11);
  digitalWrite(trigpin, LOW);

  duration = pulseIn(echopin, HIGH);

  if(duration > 0){
    dist[cnt] = duration/2*(331.5+0.61*tempera)/10000;

    res += dist[cnt];
    res -= dist[(cnt-1+num)%num];

    cnt = (cnt+1)%num;
  }

  return res;
}
double HCSR04::getdist(){
  return HCSR04::getdist(15);
}

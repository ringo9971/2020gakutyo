#include "Arduino.h"
#include "HCSR04.h"

HCSR04::HCSR04(int _echopin, int _trigpin){
  echopin = _echopin;
  trigpin = _trigpin;

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

  return duration/2*(331.5+0.61*tempera)/10000;
}
double HCSR04::getdist(){
  return HCSR04::getdist(15);
}

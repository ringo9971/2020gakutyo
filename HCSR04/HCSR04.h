#ifndef _HCSR04_h_
#define _HCSR04_h_
#include "Arduino.h"

class HCSR04{
  private:
    double duration;
    double dist[9];
    double res;
    int num;
    int cnt;
    int echopin, trigpin;
  public:
    HCSR04(int _echopin, int _trigpin);
    double getdist();
    double getdist(double tempera);
};

#endif

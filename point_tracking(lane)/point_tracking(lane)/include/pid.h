#include<ros/ros.h>
#include<iostream>

class pid{
    double P, I, D;
    double pre_error;
public:
    pid();
    double PID(double angle, double rate);
};

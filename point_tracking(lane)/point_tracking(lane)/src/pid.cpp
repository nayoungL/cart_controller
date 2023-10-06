#include "pid.h"

pid::pid()
{
    ros::NodeHandle pnh("~");
    pnh.param("PID_P",P,1.8);
    pnh.param("PID_I",I,0.0);
    pnh.param("PID_D",D,0.0);



}


double pid::PID(double angle, double rate)
{
    double error;
    double result_pid;

    error = angle;
    double steer_derivative = (error - pre_error)*rate;
    result_pid = P * error + D *steer_derivative/* + I*steer_integral*/;

    //max: 0.35, min: -0.35
    if (result_pid >= 0.35)
        result_pid = 0.35;
    if (result_pid <= -0.35)
        result_pid = -0.35;

    pre_error = error;

    return result_pid;
}

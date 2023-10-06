#include<ros/ros.h>
#include<iostream>
#include<math.h>
#include<cmath>
#include<vector>
#include<string>
#include<sstream>

#include<geometry_msgs/Point.h>
#include<geometry_msgs/Point32.h>
#include<geometry_msgs/PointStamped.h>
#include<geometry_msgs/Twist.h>

#include<visualization_msgs/MarkerArray.h>

#include<std_msgs/Int32.h>
#include<std_msgs/Int8.h>
#include<std_msgs/String.h>
#include<std_msgs/Bool.h>
#include<std_msgs/Int32MultiArray.h>

#include<sensor_msgs/PointCloud.h>

#include<pcl_ros/point_cloud.h>
#include<sensor_msgs/point_cloud_conversion.h>

#include<ackermann_msgs/AckermannDriveStamped.h>
#include<ackermann_msgs/AckermannDrive.h>

//#include "pid.h"


class point_tracking
{
    double max_lfd, min_lfd, back_x, back_y, vel_param, VL;
    double velocity;

    bool is_look_foward_point;

    double pid_P=0.5;
    double pid_I=0.0;
    double pid_D=0.0;
    double pre_error;

    bool motor_one=false;
    bool ing=false;
    int detect_node;

//subscribe
    bool is_go=false;
    std::vector<int> list_vec;
    sensor_msgs::PointCloud waypoint;
    std::string marker_node;
    std::vector<int> basket_vec;
    int lane_id;
//publish
    ackermann_msgs::AckermannDriveStamped cmd_vel;
    std_msgs::Bool frame;

    //std_msgs::String go_str;
    bool marker_detect_before=false;

public:
    point_tracking();

    double steering_angle(sensor_msgs::PointCloud way_pt);
    double velocity_linear(sensor_msgs::PointCloud way_pt);
    double PID(double angle, double rate);
    void process();

    int find_marker(std::string node, std::vector<int> list, bool& detect_before);
    bool find_basket(std::vector<int> list, std::vector<int>basket);

    //callback
    void go_sub(const std_msgs::String &msg);
    void list_sub(const std_msgs::String &msg);
    void way_sub(const sensor_msgs::PointCloud2 &msg);
    void marker_sub(const std_msgs::String &msg);
    void basket_sub(const std_msgs::Int32MultiArray::ConstPtr &msg);
    void lane_marker_sub(const std_msgs::Int8 &msg);

    //subscriber
    ros::Subscriber sub_go, sub_list;
    ros::Subscriber sub_way;
    ros::Subscriber sub_marker;
    ros::Subscriber sub_barket;
    ros::Subscriber sub_lane_marker;

    //publisher
    ros::Publisher cmd_pub;
    ros::Publisher frame_pub;
    ros::Publisher motor_pub;

};

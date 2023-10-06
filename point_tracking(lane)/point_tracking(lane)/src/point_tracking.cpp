#include "point_tracking.h"

point_tracking::point_tracking()
{   ros::NodeHandle nh;
    ros::NodeHandle pnh;

    std::string twist_topic;


    pnh.param<double>("VL", VL, 0.325);
    pnh.param<double>("velocity", velocity, 0.3);
    pnh.param<double>("back_x", back_x, -0.3);
    pnh.param<double>("back_y", back_y, 0.0);
    pnh.param<double>("max_lfd", max_lfd, 1.5);
    pnh.param<double>("min_lfd", min_lfd, 1.0);
    pnh.param<double>("vel_param", vel_param, 5.0);

    //publish
    cmd_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd",10);
    frame_pub = nh.advertise<std_msgs::Bool>("/frame",10);
    motor_pub = nh.advertise<std_msgs::Int8>("/motor_index",10);

    //subscriber
//socket car start sub
    sub_go = nh.subscribe("/socket_go",10,&point_tracking::go_sub,this);
//socket shopping list sub
    sub_list = nh.subscribe("/socket_list",10,&point_tracking::list_sub,this);
//lane detection sub
    sub_way = nh.subscribe("/waypoint",10,&point_tracking::way_sub,this);
//marker detection node sub
    sub_marker = nh.subscribe("/marker",10,&point_tracking::marker_sub,this);
//basket check sub
    sub_barket = nh.subscribe("/basket_check",10,&point_tracking::basket_sub,this);
    
    sub_lane_marker = nh.subscribe("/lane_marker",10,&point_tracking::lane_marker_sub,this);


    is_look_foward_point = false;
    frame.data=true;

}


///////////////////////////////////subscribe////////////////////////////////////
void point_tracking::go_sub(const std_msgs::String& msg){
    is_go=true;
}

void point_tracking::list_sub(const std_msgs::String& msg){
//msg save
    std::string node_str;
    node_str=msg.data;

//string to vector
    std::vector<std::string> str_vec;
    std::istringstream iss(node_str);
    std::string str_buf;
    char separator=' ';
    while(getline(iss, str_buf, separator)){
        str_vec.push_back(str_buf);
    }
//string vector to ing vector
    for(int i=0; i<str_vec.size(); i++){
	int num=0;
	std::stringstream ss;
	ss << str_vec[i];
	ss >> num;
        list_vec.push_back(num);
        std::cout << num << " ";
    }
    std::cout << std::endl;

//order : cider(0), coke(1), candy(2)
}

void point_tracking::way_sub(const sensor_msgs::PointCloud2& msg)
{
    sensor_msgs::convertPointCloud2ToPointCloud(msg, waypoint);

    for(int i=0; i<waypoint.points.size(); i++){
	float point_x=waypoint.points.at(i).x;
	float point_y=waypoint.points.at(i).y;
	waypoint.points.at(i).x=point_y;
	waypoint.points.at(i).y=point_x;
    }
}

void point_tracking::marker_sub(const std_msgs::String &msg){
    marker_node=msg.data;
// no marker
// start_node
// cider_node
// coke_node
// candy_node
}


void point_tracking::basket_sub(const std_msgs::Int32MultiArray::ConstPtr& msg){
//msg : coke=0, cider=1, candy=2

    basket_vec.resize(3, 0);

    int coke_cnt=0;
    int cider_cnt=0;
    int candy_cnt=0;
    for(std::vector<int>::const_iterator it= msg->data.begin(); it!=msg->data.end(); ++it){
	if( *it == 0 ){
	    ++coke_cnt;
	    basket_vec[1]=coke_cnt;
	}
	else if ( *it == 1 ){
	    ++cider_cnt;
	    basket_vec[0]=cider_cnt;
	}
	else{
	    ++candy_cnt;
	    basket_vec[2]=candy_cnt;
	}
    }
//basket : cider(0), coke(1), candy(2)
}

void point_tracking::lane_marker_sub(const std_msgs::Int8 &msg){
    lane_id=msg.data;
}



///////////////////////function////////////////////////
int point_tracking::find_marker(std::string node, std::vector<int> list, bool& detect_before){

    int marker_result;
    if(lane_id==1){
	    if(list.at(0)>0){
	        marker_result = 0;
	    }
	    else{
	        marker_result = 100;
	    }
    }
    else if(lane_id==1){
	
	    if(list.at(1)>0){
	        marker_result = 1;
	    }
	    else{
	        marker_result = 100;
	    }

    }
    else if(lane_id==2){
	
	    if(list.at(2)>0){
	        marker_result = 2;
	    }
	    else{
	       marker_result = 100;
	    }

    }
    else if(lane_id==3){
	
	    marker_result=10;

    }
    else{
	    marker_result = 100;    
    }

    if(marker_result != 100 && detect_before==false){
	detect_before=true;
	return marker_result;
    }
    else{
	return 100;
    }


    // int marker_result;
    // if(node == "cider_node"){
	
	// if(list.at(0)>0){
	//     marker_result = 0;
	// }
	// else{
	//     marker_result = 100;
	// }

    // }
    // else if(node == "coke_node"){
	
	// if(list.at(1)>0){
	//     marker_result = 1;
	// }
	// else{
	//     marker_result = 100;
	// }

    // }
    // else if(node == "candy_node"){
	
	// if(list.at(2)>0){
	//     marker_result = 2;
	// }
	// else{
	//     marker_result = 100;
	// }

    // }
    // else if(node == "start_node"){
	
	// marker_result=10;

    // }
    // else{
	// marker_result = 100;    
    // }

    // if(marker_result != 100 && detect_before==false){
	// detect_before=true;
	// return marker_result;
    // }
    // else{
	// return 100;
    // }
}

bool point_tracking::find_basket(std::vector<int> list, std::vector<int>basket){
    bool check=true;

    int diff;
    int index;
    for(int i=0; i<list.size(); i++){
	if(list[i] != basket[i]){
	    check=false;
	    index=i;
	    diff=std::abs(list[i]-basket[i]);
	}
    }

    if(check==false){
        std_msgs::Int8 motor_index;
	motor_index.data=index;
	for(int i=0; i<diff; i++){
	    motor_pub.publish(motor_index);
	    ros::Duration(4.0).sleep();
	}
    }
    std::cout << "check : " << check << std::endl;
    return check;
}


/////////////////////////////cmd_vel//////////////////////////////////
double point_tracking::steering_angle(sensor_msgs::PointCloud way_pt)
{
    double min_dist = 10;
    double min_index = 0;
    double steering = 0;
    double dis = 0;
    float lfd = 1;
    bool is_waypoint=false;

    for(int i = 0; i<way_pt.points.size(); i++)
    {
        double dx = back_x - way_pt.points.at(i).x;
        double dy = back_y - way_pt.points.at(i).y;

        double dist = sqrt(dx*dx + dy*dy);
        if(dist<min_dist)
        {
            min_dist = dist;
            min_index = i;
        }

	if(way_pt.points.at(i).x==-1 && way_pt.points.at(i).y==-1){

	    is_waypoint=false;
	}
	else{
	    is_waypoint=true;
	}
    }

    if(lfd < min_lfd)
    {
        lfd = min_lfd;
    }
    else if(lfd > max_lfd)
    {
        lfd = max_lfd;
    }

    double dx, dy=0;

    lfd=0.0;
    for(int i = 0; i<way_pt.points.size(); i++)
    {
        dx = way_pt.points.at(i).x - back_x;
        dy = way_pt.points.at(i).y;

        if(dx > 0)
        {
            dis = sqrt(pow(dx,2) + pow(dy,2));

            if(dis>=lfd)
            {
                is_look_foward_point = true;
                break;
            }
        }
    }

    double theta = atan2(dy,dx);


    if(is_look_foward_point == true)
    {

        double eta = atan2((2*VL*sin(theta)),lfd);
        steering=eta;

    }
    else
    {
        ROS_INFO("no found forwad point");
    }

    if(is_waypoint==false){
	steering=0;
    }

    return (-1)*steering;
}

double point_tracking::velocity_linear(sensor_msgs::PointCloud way_pt){
    
    double vel;

    is_go=true;
    list_vec.resize(3,0);
    basket_vec.resize(3,0);
    list_vec[0]=2;
    list_vec[1]=0;
    list_vec[2]=0;
    basket_vec[0]=2;
    basket_vec[0]=0;
    basket_vec[0]=0;

    if(is_go == false){
	std::cout << "ready" << std::endl;
	vel=0;
    }
    else{
	std::cout << "start" << std::endl;
	
	detect_node=find_marker(marker_node, list_vec, marker_detect_before);
	std::cout << marker_detect_before << std::endl;
	std::cout <<detect_node << std::endl;
	if(marker_detect_before == true){
	    if(detect_node==10 && ing==true){
		std::cout << "end" << std::endl;
		vel=0;
	    }
	    else if(detect_node >= 0 && detect_node <=2){
		std::cout << "marker stop "<< detect_node << std::endl;
		vel=0;
	    }
	    else{
		std::cout << "marker else" << std::endl;
           	for(int i=0; i<way_pt.points.size(); i++){
                    if(way_pt.points.at(i).x==-1 && way_pt.points.at(i).y==-1){
                        vel=velocity-0.1;
                    }
                    else{
                        vel=velocity;
                    }
                }
		marker_detect_before=false;
	    }
	}
	else{
	    std::cout << "keep going" << std::endl;
            for(int i=0; i<way_pt.points.size(); i++){
                if(way_pt.points.at(i).x==-1 && way_pt.points.at(i).y==-1){
                    vel=velocity-0.1;
                }
                else{
                    vel=velocity;
                }
            }
	}
    }
	    

    return vel;
}


double point_tracking::PID(double angle, double rate){
    double error;
    double result_pid;

    error = angle;
    double steer_derivative = (error - pre_error)*rate;
    result_pid = pid_P * error + pid_D *steer_derivative/* + I*steer_integral*/;

    if (result_pid >= 0.35)
        result_pid = 0.35;
    if (result_pid <= -0.35)
        result_pid = -0.35;
    pre_error = error;

    return result_pid;
}


////////////////////////////run////////////////////////////
void point_tracking::process()
{
    std::cout << std::endl;
    std::cout << "steering" << std::endl;
    double angle=steering_angle(waypoint);
    angle=PID(angle, 10);

    std::cout << "velocity" << std::endl;
    double linear=velocity_linear(waypoint);

    cmd_vel.drive.steering_angle = angle;
    cmd_vel.drive.speed = linear;
    cmd_pub.publish(cmd_vel);


    if(linear == 0){
	if(motor_one == false){
	    std_msgs::Int8 motor_index;
	    motor_index.data=detect_node;
	    motor_one=true;

	    for(int i=0; i<list_vec[detect_node]; i++){
		ing=true;
		motor_pub.publish(motor_index);
		std::cout <<"motor publish" << std::endl;
		ros::Duration(4.0).sleep();
		std::cout << "sleep" << std::endl;
	    }
	}
	std::cout << "frame publish" << std::endl;
	ros::Duration(3.0).sleep();
	frame_pub.publish(frame);

	std::cout << "check basket" << std::endl;
	if(find_basket(list_vec, basket_vec) == true){
	    marker_detect_before=false;
	    motor_one=false;
	}
    }

}

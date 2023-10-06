#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "iostream"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "vector"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/point_field_conversion.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include <std_msgs/String.h>

struct l_pt{
    int label=0;
    int x;
    double slope;
    int cnt;
};
cv_bridge::CvImagePtr rawImagePtr, pubImagePtr;
cv::Mat rawImage;

bool is_waypoint;
std::vector<geometry_msgs::Point32> waypoint;
geometry_msgs::Point32 pre_point;



bool cmp(l_pt& a, l_pt&b){
    return a.x<b.x;
}

void subImgCallback(const sensor_msgs::CompressedImage& subImgMsgs){
    if(subImgMsgs.data.size()){
        rawImagePtr=cv_bridge::toCvCopy(subImgMsgs, sensor_msgs::image_encodings::BGR8);
        rawImage=rawImagePtr->image;
    }
    cv::imshow("origin", rawImage);
    cv::waitKey(1);
}

bool process(cv::Mat test_img, std::vector<geometry_msgs::Point32>& waypoint){

///roi
    cv::Mat roi_img;
    cv::Rect tmp(0, 0, test_img.cols, test_img.rows);
    cv::Rect r(0, 50, test_img.cols, 250);
    roi_img=test_img(r&tmp);

///canny
    cv::Mat gaussian_img;
    cv::GaussianBlur(roi_img, gaussian_img, cv::Size(5,5),3);

    cv::Mat canny_img;
    cv::Canny(gaussian_img, canny_img, 80, 125);

///color
    cv::Mat hsv_img;
    cv::cvtColor(test_img, hsv_img, cv::COLOR_BGR2HSV);

    cv::Mat yellow_mask, yellow_img;
    cv::Scalar lower_yellow=cv::Scalar(20,70,150);
    cv::Scalar upper_yellow=cv::Scalar(35,180,255);
    cv::inRange(hsv_img, lower_yellow, upper_yellow, yellow_mask);
    cv::bitwise_and(test_img, test_img, yellow_img, yellow_mask);
    cv::Mat white_mask, white_img;
    cv::Scalar lower_white=cv::Scalar(0,0,200);
    cv::Scalar upper_white=cv::Scalar(120,40,255);
    cv::inRange(hsv_img, lower_white, upper_white, white_mask);
    cv::bitwise_and(test_img, test_img, white_img, white_mask);

    cv::Mat yellow_binary_img;
    cv::cvtColor(yellow_img, yellow_binary_img, cv::COLOR_BGR2GRAY);
    cv::threshold(yellow_binary_img, yellow_binary_img, 125, 255, cv::THRESH_BINARY);
    cv::Mat white_binary_img;
    cv::cvtColor(white_img, white_binary_img, cv::COLOR_BGR2GRAY);
    cv::threshold(white_binary_img, white_binary_img, 125, 255, cv::THRESH_BINARY);
    //cv::Mat binary_img=white_binary_img+yellow_binary_img;
    cv::Mat binary_img=white_binary_img;

    binary_img=binary_img(r&tmp);
    cv::dilate(binary_img, binary_img, cv::Mat::ones(cv::Size(3,3), CV_8UC1), cv::Point(-1, -1));
    cv::dilate(binary_img, binary_img, cv::Mat::ones(cv::Size(3,3), CV_8UC1), cv::Point(-1, -1));

    cv::dilate(binary_img, binary_img, cv::Mat::ones(cv::Size(3,3), CV_8UC1), cv::Point(-1, -1));


///sum
    cv::Mat sum_img;
    cv::bitwise_and(canny_img, binary_img, sum_img);


///transformation
    cv::Mat bev_img;
    cv::Point2f src_vertices[4];
    src_vertices[0]=cv::Point(0, 130);
    src_vertices[1]=cv::Point(130, 0);
    src_vertices[2]=cv::Point(510, 0);
    src_vertices[3]=cv::Point(640, 130);
    cv::Point2f dst_vertices[4];
    dst_vertices[0]=cv::Point(0, 250);
    dst_vertices[1]=cv::Point(0, 0);
    dst_vertices[2]=cv::Point(640, 0);
    dst_vertices[3]=cv::Point(640, 250);
    cv::Mat matrix=cv::getPerspectiveTransform(src_vertices, dst_vertices);
    cv::warpPerspective(sum_img, bev_img, matrix, roi_img.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);

    cv::Mat re_sum_img;
    cv::resize(bev_img, re_sum_img, cv::Size(640, 500));

///line
    cv::Mat line_img;
    cv::cvtColor(re_sum_img, line_img, cv::COLOR_GRAY2BGR);
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(re_sum_img, lines, 1, (CV_PI/180), 30, 10, 20);

    std::vector<l_pt> line_pt;
    int line_range=30;
    for(int i=0; i<lines.size(); i++){
        cv::Vec4i l=lines.at(i);
        float dist=std::sqrt(std::pow(l[0]-l[2], 2)+std::pow(l[1]-l[3], 2));
	    if(dist>=160){
            cv::line( line_img, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 2,8);
            l_pt pt;
            pt.x=(l[0]+l[2])/2;
            if(l[0]==l[2]){
                pt.slope=1000;
            }
            else{
                pt.slope=std::abs((double)(l[1]-l[3]))/std::abs((double)(l[0]-l[2]));
            }
            line_pt.push_back(pt);
	    }
    }

///line classification
    int l_cnt=1;
    double l_slope_avg;
    double l_x_avg;
    int label=0;
    int first_pt;
    std::sort(line_pt.begin(), line_pt.end(), cmp);
    std::vector<l_pt> line_center_pt;
    for(int i=0; i<line_pt.size(); i++){
        l_pt pt, center_pt;
        pt=line_pt.at(i);
        if(i==0){
            pt.label=label;
            first_pt=pt.x;

            center_pt.x=pt.x/(double)l_cnt;
            center_pt.label=label;
            center_pt.slope=pt.slope/(double)l_cnt;
	    center_pt.cnt=l_cnt;
            line_center_pt.push_back(center_pt);
            l_cnt++;
        }
        else{
            if(std::abs(first_pt-pt.x)<=100){
                pt.label=label;

                center_pt.x=(line_center_pt.at(label).x*(l_cnt-1)+pt.x)/(double)l_cnt;
                center_pt.label=label;
                center_pt.slope=(line_center_pt.at(label).slope*(l_cnt-1)+pt.slope)/(double)l_cnt;
		center_pt.cnt=l_cnt;
                line_center_pt.at(label)=center_pt;
                l_cnt++;
            }
            else{
                label++;
                pt.label=label;
                first_pt=pt.x;

                l_cnt=1;
                center_pt.x=pt.x/(double)l_cnt;
                center_pt.label=label;
                center_pt.slope=pt.slope/(double)l_cnt;
		center_pt.cnt=l_cnt;
                line_center_pt.push_back(center_pt);
                l_cnt++;
            }
        }
        line_pt.at(i)=pt;
    }


///line center
    int center_x;
    int x_sum=0;
    double min_slope=100;
    int final_cnt=0;
    for(int i=0; i<line_center_pt.size(); i++){
	std::cout << line_center_pt.at(i).cnt << " ";
	if(line_center_pt.at(i).cnt>=3){
	    std::cout << line_center_pt.at(i).x << " ";
	    final_cnt++;
            x_sum+=line_center_pt.at(i).x; 
            if(min_slope>line_center_pt.at(i).slope){
                min_slope=line_center_pt.at(i).slope;
            }
	}
    }
    std::cout << std::endl;
    if(line_center_pt.size()>0){
    if(final_cnt==0){
	center_x=line_center_pt.at(0).x;
    }
    else {
	center_x=x_sum/final_cnt;
    }
    std::cout << center_x << std::endl;
    std::cout << final_cnt << std::endl;
    std::cout << min_slope << std::endl;
    if(min_slope>=3){
        if(final_cnt==1){
            center_x=center_x-250;
        }
        else{
	    if(center_x>400){
		center_x=center_x-250;
	    }
	    else{
		center_x=center_x;
	    }
	}
    }
    else if(min_slope>=2.6){
        if(final_cnt==1){
            center_x=center_x-250;
        }
        else{
	    if(center_x>400){
		center_x=center_x-250;
	    }
	    else{
		center_x=center_x;
	    }
	}
	center_x=center_x+30;
    }
    else if(min_slope>=1.2){
        center_x=center_x+60; 
	if(center_x>=400){
	    center_x=400;
	}
    }
    else{
	if(center_x>=300){
	    center_x=center_x-100;
	}
        center_x=center_x-30; 
    }

    if(center_x<0){
        center_x=0;
    }
    if(center_x>=640){
        center_x=640-1;
    }

    geometry_msgs::Point32 center;
    center.x=center_x;
    center.y=300;
    waypoint.push_back(center);

    cv::circle(line_img, cv::Point(center.x, center.y), 8, cv::Scalar(0,255,0), 3);
    std::cout << center << std::endl;
    }

    //cv::imshow("roi", roi_img);
    //cv::imshow("sum", sum_img);
    //cv::imshow("bev", bev_img);
    cv::imshow("line", line_img);

    if(waypoint.size()>0){
	is_waypoint=true;

    }
    else{
	is_waypoint=false;
	std::cout << "no waypoint" << std::endl;
    }
    return is_waypoint;
}


int main(int argc, char** argv){

    ros::init(argc, argv, "lane_detection");
    ros::NodeHandle nh;

    ros::Subscriber subImage=nh.subscribe("camera_front/usb_cam/image_raw/compressed",1, subImgCallback);
    ros::Publisher PubPoint=nh.advertise<sensor_msgs::PointCloud2> ("/waypoint",1);

    while(ros::ok()){

        std::vector<geometry_msgs::Point32> waypoint;
        sensor_msgs::PointCloud waypoint_output1;
        sensor_msgs::PointCloud2 waypoint_output2;
        if(!rawImage.empty()){

            if(process(rawImage, waypoint)){
//                std::cout << "process" << std::endl;

                if(waypoint[0].x>=320){waypoint[0].x=waypoint[0].x-320;}
                else{ waypoint[0].x=waypoint[0].x-320; }
                waypoint[0].x=waypoint[0].x/1000;
                waypoint[0].y=(500-waypoint[0].y)/1000;
                waypoint_output1.points.push_back(waypoint[0]);



                waypoint.clear();
            }
            else{
                //std::cout << "no waypoint" << std::endl;

                geometry_msgs::Point32 no_point;
                no_point.x=-1;
                no_point.y=-1;
                waypoint_output1.points.push_back(no_point);

            }

            sensor_msgs::convertPointCloudToPointCloud2(waypoint_output1, waypoint_output2);
            PubPoint.publish(waypoint_output2);

            waypoint_output1.points.clear();

     	cv::waitKey(1);

        }
        else{
            std::cout << "empty" << std::endl;
        }
        ros::spinOnce();

    }
}



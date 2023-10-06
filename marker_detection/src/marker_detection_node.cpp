#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "iostream"
#include "vector"
//#include "opencv2/aruco/dictionary.hpp"
//#include "opencv2/aruco.hpp"
#include "opencv2/core/hal/hal.hpp"
#include <std_msgs/String.h>


cv_bridge::CvImagePtr rawImagePtr, pubImagePtr;
cv::Mat rawImage;

void subImgCallback(const sensor_msgs::CompressedImage& subImgMsgs){
    if(subImgMsgs.data.size()){
        rawImagePtr=cv_bridge::toCvCopy(subImgMsgs, sensor_msgs::image_encodings::BGR8);
        rawImage=rawImagePtr->image;
    }
}



static unsigned char DICT_6X6_1000_BYTES[][4][5]=
{
    {
        {30,61,216,42,6},{227,186,70,49,9},{101,65,187,199,8},{152,198,37,220,7}
    },
    {
        {14,251,163,137,1},{215,230,24,5,14},{137,28,93,247,0},{122,1,134,126,11}
    },
    {
        {21,144,126,172,13},{236,105,87,80,6},{179,87,224,154,8},{96,174,169,99,7}
    },
    {
        {201,27,48,105,14},{66,50,75,222,12},{121,96,205,137,3},{55,189,36,196,2}
    },
    {
        {214,7,214,225,5},{164,203,75,191,2},{168,118,190,6,11},{79,213,45,50,5}
    }
};

cv::Mat dictionary = cv::Mat(250, (6 * 6 + 7) / 8, CV_8UC4, (uchar*)DICT_6X6_1000_BYTES);


cv::Mat get_bytefrombits(const cv::Mat &bits){
    int nbytes=(bits.cols * bits.rows +8-1)/8;

    cv::Mat bytelist(1, nbytes, CV_8UC1, cv::Scalar::all(0));
    unsigned char current_bit=0;
    int current_byte=0;

    uchar* rot0=bytelist.ptr();

    for(int r=0; r<bits.rows; r++){
        for(int c=0; c<bits.cols; c++){
            rot0[current_byte] <<=1;

            rot0[current_byte] |= bits.at<uchar>(r,c);

            current_bit++;
            if(current_bit==8){
                current_bit=0;
                current_byte++;
            }
        }
    }
    return bytelist;
}



bool identify(const cv::Mat &onlyBits, int &idx, int &rotation){
    int marker_size=6;

    cv::Mat candidateBytes=get_bytefrombits(onlyBits);

    idx=-1;

    int MinDistance=marker_size*marker_size+1;
    rotation=-1;

    for(int m=0; m<5; m++){
        for(unsigned int r=0; r<4; r++){
            int currentHamming=cv::hal::normHamming(dictionary.ptr(m)+r*candidateBytes.cols, candidateBytes.ptr(), candidateBytes.cols);

            if(currentHamming<MinDistance){

                MinDistance=currentHamming;
                rotation=r;
                idx=m;

            }
        }
    }

    return idx!=-1;
}



int process(cv::Mat img){

    cv::Mat gray_img;
    cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);

    cv::Mat binary_img;
    cv::adaptiveThreshold(gray_img, binary_img, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 191, 7);

//    //marker 외곽 찾기
    cv::Mat contour_img=binary_img.clone();
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(contour_img, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

//    //marker만 따로 저장
    std::vector<std::vector<cv::Point2f>> marker;
    std::vector<cv::Point2f> approx;
    for(int i=0; i<contours.size(); i++){
        cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.05, true);
        if(approx.size()==4 && std::fabs(cv::contourArea(cv::Mat(approx)))>1000 &&
                std::fabs(cv::contourArea(cv::Mat(approx)))<50000 && cv::isContourConvex(cv::Mat(approx))){
            cv::drawContours(img, contours, i, cv::Scalar(0,255,0), 1, cv::LINE_AA);
            std::vector<cv::Point2f> points;
            for(int j=0; j<4; j++){
                points.push_back(cv::Point2f(approx[j].x, approx[j].y));
            }
            cv::Point v1=points[1]-points[0];
            cv::Point v2=points[2]-points[0];
            double o=(v1.x*v2.y)-(v1.y*v2.x);
            if(o<0.0){
                std::swap(points[1], points[3]);
            }
            marker.push_back(points);
        }
    }


//    //marker 외곽 검은색 부분으로 marker부분 구분
    std::vector<std::vector<cv::Point2f>> detected_markers;
    std::vector<cv::Mat> detected_marker_img;
    std::vector<cv::Point2f> square_points;
    cv::Mat marker_img;
    int marker_length=80;
    square_points.push_back(cv::Point2f(0,0));
    square_points.push_back(cv::Point2f(marker_length-1,0));
    square_points.push_back(cv::Point2f(marker_length-1,marker_length-1));
    square_points.push_back(cv::Point2f(0,marker_length-1));

    for(int i=0; i<marker.size(); i++){
        std::vector<cv::Point2f> m=marker[i];
        cv::Mat matrix=cv::getPerspectiveTransform(m, square_points);
        cv::warpPerspective(gray_img, marker_img, matrix, cv::Size(marker_length, marker_length));

        cv::threshold(marker_img, marker_img, 125, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

        int cell_size=marker_img.rows/8;
        int white_cnt=0;
        for(int y=0; y<8; y++){
            int inc=7;
            if(y==0 || y==7){
                inc=1;
            }
            for(int x=0; x<8; x+=inc){
                int cell_x=x*cell_size;
                int cell_y=y*cell_size;
                cv::Mat cell=marker_img(cv::Rect(cell_x, cell_y, cell_size, cell_size));

                int cell_cnt=cv::countNonZero(cell);
                if(cell_cnt>(cell_size*cell_size)/2){
                    white_cnt++;
                }
            }
        }
        if(white_cnt==0){
            detected_markers.push_back(m);
            cv::Mat temp_img=marker_img.clone();
            detected_marker_img.push_back(temp_img);
        }
    }



//    //marker 내부 bit로 정보 확인
    std::vector<cv::Mat> bit_matrix;
    for(int i=0; i<detected_markers.size(); i++){
        cv::Mat temp_img=detected_marker_img[i];
        cv::Mat temp_bit=cv::Mat::zeros(6,6, CV_8UC1);
        int cell_size=temp_img.rows/8;
        for(int y=0; y<6; y++){
            for(int x=0; x<6; x++){
                int cell_x=(x+1)*cell_size;
                int cell_y=(y+1)*cell_size;
                cv::Mat cell=temp_img(cv::Rect(cell_x, cell_y, cell_size, cell_size));

                int cell_cnt=cv::countNonZero(cell);
                if(cell_cnt>(cell_size*cell_size)/2){
                    temp_bit.at<uchar>(y,x)=1;
                }
            }
        }
        bit_matrix.push_back(temp_bit);
    }


    std::vector<int> marker_id;
    std::vector<std::vector<cv::Point2f>> final_detected_marker;
    for(int i=0; i<detected_markers.size(); i++){
        cv::Mat bitmatrix=bit_matrix[i];
        std::vector<cv::Point2f> m=detected_markers[i];

        int rotation;
        int id;
        if(!identify(bitmatrix, id, rotation)){
            std::cout << "no" << std::endl;
        }
        else{
            if(rotation !=0){
                std::rotate(m.begin(), m.begin()+4-rotation, m.end());
            }
//            std::cout << id << " ";
            marker_id.push_back(id);
            final_detected_marker.push_back(m);

        }
    }
    std::cout << std::endl;


    int current_marker;
    std::vector<float> marker_size;
    if(marker_id.size()>0){
        for(int i=0; i<marker_id.size(); i++){
            std::vector<cv::Point2f> m=detected_markers.at(i);
            float size=std::sqrt((m[0].x-m[1].x)*(m[0].x-m[1].x)+(m[0].y-m[1].y)*(m[0].y-m[1].y));
            marker_size.push_back(size);
        }

        float min=300;
        int min_index;
        for(int i=0; i<marker_size.size(); i++){
            if(min>marker_size.at(i)){
                min=marker_size.at(i);
                min_index=i;
            }
        }

        current_marker=marker_id.at(min_index);
    }
    else{
        std::cout << "no marker detect" << std::endl;
        current_marker=100;
    }

    return current_marker;



//    bool find=false;
//    int find_id=2;
//    if(marker_id.size()>0){
//        for(int i=0; i<marker_id.size(); i++){
//            if(marker_id.at(i) == find_id){
//                std::vector<cv::Point2f> m=detected_markers.at(i);
//                float min_d=std::sqrt((m[0].x-m[1].x)*(m[0].x-m[1].x)+(m[0].y-m[1].y)*(m[0].y-m[1].y));
//                for(int i=1; i<3; i++){
//                    float d=std::sqrt((m[i].x-m[i+1].x)*(m[i].x-m[i+1].x)+(m[i].y-m[i+1].y)*(m[i].y-m[i+1].y));
//                    if(min_d > d){
//                        min_d=d;
//                    }
//                }

//                if(min_d>30){

////                    std::cout << "gggggggggggggggggggggggggggoooooooooooooooo" << std::endl;
//                    find=true;
//                    std::cout << find_id << " " << min_d << std::endl;
//                }
//                else{
//                    std::cout << "no id " << min_d << std::endl;
//                    find=false;
//                }
//            }
//            else{
//                std::cout << "no id " << find_id << std::endl;
//            }
//        }
//    }
//    else{
//        std::cout << "no" << std::endl;
//        find=false;
//    }
//    cv::imshow("binary", binary_img);
//    return find;
}


    int main(int argc, char** argv){

        ros::init(argc, argv, "marker_detection");
        ros::NodeHandle nh;

        ros::Subscriber subImage=nh.subscribe("camera_front/usb_cam/image_raw/compressed",1, subImgCallback);
        ros::Publisher pubNode_Go=nh.advertise<std_msgs::String>("/marker", 1);

        std_msgs::String go;
        std::stringstream go_ss;

        while(ros::ok()){

            if(!rawImage.empty()){

                int node_id=process(rawImage);
                std::cout << node_id << std::endl;
                cv::imshow("original", rawImage);

                if(node_id==100){
                    go_ss << "no marker";
                }
                else if(node_id==0){
                    go_ss << "start_node";
                }
                else if(node_id==1){
                    go_ss << "cider_node";
                }
                else if(node_id==2){
                    go_ss << "coke_node";
                }
                else if(node_id==3){
                    go_ss << "candy_node";
                }
//                if(pub_bool){
//                    go_ss << "stop";
//                }
//                else{
//                    go_ss << "go";
//                }
                go.data=go_ss.str();
                pubNode_Go.publish(go);

                go_ss.str("");

                if(cv::waitKey(25)==27){
                    break;
                }

            }
            else{
//                std::cout << "empty" << std::endl;
            }
            ros::spinOnce();

        }
    }




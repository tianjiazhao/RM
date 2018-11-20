#pragma once
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "Armordetector.h"

class ImageConsProd
{
public:
    ImageConsProd(int fd_car){
    fd2car = fd_car;
}

    void ImageProducer();
    void ImageConsumer();
public:
    int fd2car;
    ArmorDetector armor;
    //camera
    double camD[9]={
        540.457180838601740,0,338.656973717591140,
        0,536.319366436046830,261.989712848073850,
        0,   0,  1};
    cv::Mat camera_matrix=cv::Mat(3,3,CV_64FC1,camD);
    double distCoeffD[5]={0.087925620063487 , -0.208430407906815 ,
                          -0.006207515746944 , -0.000310843154146 , 0.000000000000000};
    cv::Mat distortion_coefficients = cv::Mat(5,1,CV_64FC1,distCoeffD);
    double min_detect_distance = 50.0, max_detect_distance = 600.0;
    double bullet_speed = 25;  //假设速度为25m/s
    double current_ptz_angle = 0; //没多大用
    cv::Point2f offset;
    //
    const double offset_anlge_x = 0;
    const double offset_anlge_y = 0; //实际调车产生的差距
};



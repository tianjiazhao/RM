#include "Imageconsprod.h"
#include "Anglesolver.h"
#include "RMVideoCapture.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <queue>
#include "Predictor.h"
#include "serial.h"
#define BUFFER_SIZE 1
volatile unsigned int prodIdx;
volatile unsigned int consIdx;
struct Imgdata
{
    cv::Mat img;
    unsigned int frame;   //帧数
};

Imgdata data[BUFFER_SIZE];
#define USE_VIDEO

void ImageConsProd::ImageProducer()
{
#ifdef USE_VIDEO
    cv::VideoCapture cap("3.avi");
    if(!cap.isOpened())
        return ;
#else
//    cv::VideoCapture cap(1);
//    cap.set(CV_CAP_PROP_FRAME_WIDTH,1080);
//    cap.set(CV_CAP_PROP_FRAME_HEIGHT,720);
    RMVideoCapture cap("/dev/video1", 3);
    cap.setVideoFormat(640, 720, 1);
    cap.setExposureTime(0, 180);//settings->exposure_time);
    cap.startStream();
    cap.info();
#endif
    while (1)
    {
        while (prodIdx-consIdx>=BUFFER_SIZE);
        cap >> data[prodIdx % BUFFER_SIZE].img;
#ifndef USE_VIDEO
        data[prodIdx % BUFFER_SIZE].frame = cap.getFrameCount();
#endif
        ++prodIdx;
    }
}

void ImageConsProd::ImageConsumer()
{

    const	Mat _template = imread("template.bmp");
    const	Mat _template_small = imread("small_template.bmp");
    armor.initTemplate(_template,_template_small);
    //angle
    AngleSolver solver(camera_matrix, distortion_coefficients ,216, 54 ,1.0,50.0,600.0);

    const double barrel_ptz_offset_y = 0;
    const double ptz_camera_y = 46.5;   //46.5
    const double ptz_camera_z = 72;   //72
//    const double overlapa_dist;

//    double theta = -atan((ptz_camera_y + barrel_ptz_offset_y)/overlapa_dist);
    double theta = 0;
    double r_data[] = {1,0,0,  0,cos(theta),-sin(theta),  0,sin(theta),cos(theta)};
    double t_data[] = {0,ptz_camera_y,ptz_camera_z};
    Mat _t_camera_ptz(3,1,CV_64FC1,t_data);
    Mat _r_camera_ptz(3,3,CV_64FC1,r_data);
    solver.Init_RelationPtzCarmera(_r_camera_ptz, _t_camera_ptz, barrel_ptz_offset_y);

    AngleFactory angle_sovler;
    angle_sovler.Init_SetArmorSize(216, 54, AngleFactory::TARGET_ARMOR);
    angle_sovler.Init_SetArmorSize(124,54,AngleFactory::TARGET_SAMLL_ATMOR);

    //predict
    Predictor predict;
    // loop
    int frame_number;
    cv::Mat src;

    while (1)
    {

        while(prodIdx - consIdx == 0);
        data[consIdx % BUFFER_SIZE].img.copyTo(src);
        frame_number = data[prodIdx % BUFFER_SIZE].frame;

        ++consIdx;
//        cv::resize(src,src,Size(640,480));
        cv::RotatedRect rect;
        double angle_x = 0.0,angle_y = 0.0;
        double send_data[3]={0};
        angle_sovler.setSlover(&solver);
#ifdef USE_VIDEO
        double t;
        t=(double)cvGetTickCount();
#endif
        rect = armor.getArmor_Final(src);
#ifdef USE_VIDEO
        t=(double)cvGetTickCount()-t;
#endif

//        printf("used time is %gms\n",(t/(cvGetTickFrequency()*1000)));

        bool is_Small = armor.Armor_isSmall();
        AngleFactory::TargetType type = is_Small ? AngleFactory::TARGET_SAMLL_ATMOR : AngleFactory::TARGET_ARMOR;
//        if(type == AngleFactory::TARGET_SAMLL_ATMOR)
//        {
//        std::cout<<"small:"<<std::endl;
//        }
        if(angle_sovler.getAngle(rect, type, angle_x, angle_y,bullet_speed ,current_ptz_angle,offset ))
        {
//            cout<<"angle:"<<angle_x<<endl<<endl;

//            predict.setRecord(angle_x, frame_number);
//            double angle_x_predict = predict.predictor(frame_number + 1.0);
//            send_data[0] = (angle_x_predict +offset_anlge_x )*100;
//            send_data[1] = (angle_y + offset_anlge_y)*100;
//            send_data[2] = 1;
                        std::cout<<"angle_x: "<<angle_x*100<<endl;
                        std::cout<<"angle_y: "<<angle_y*100<<endl;
                        send_data[0] = (angle_y)*100;
                        send_data[1] = (angle_x)*100;
                        send_data[2] = 1;
            //send data
            SendData(fd2car,send_data);
        }
        else {
            send_data[2] = 0;
            SendData(fd2car,send_data);
        }

        cv::waitKey(1);

    }
}

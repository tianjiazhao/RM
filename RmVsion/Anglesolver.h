#ifndef ANGLESOLVER_H
#define ANGLESOLVER_H
#include "opencv2/core/core.hpp"
#include<iostream>
class RectPnPSolver
{
public:
    RectPnPSolver(const cv::Mat & cam_matrix, const cv::Mat & dist_coeff,double target_width = 0,double target_height = 0)
    {
        cam_matrix.copyTo(camera_matrix);
        dist_coeff.copyTo(distortion_coeff);

        width_target = target_width;
        height_target = target_height;
    }
    void   getTarget2dPoint(const cv::RotatedRect & rect,
                            std::vector<cv::Point2f> & target2d);      //找到图像中对应的4个二维坐标
    void   solvePnP4Point(const std::vector<cv::Point2f> &point2d,cv::Mat & rot,cv::Mat & trans );        //转化为世界坐标
    void   setTargetSize(double width, double height)
    {
        width_target = width;
        height_target = height;
    }

public:
    cv::Mat camera_matrix;  //相机内参
    cv::Mat distortion_coeff;
    double width_target;
    double height_target;


};
/**
* @brief 得到的云台转角
* @param rect 输入,目标矩形位置
* @param angle_x 输出,云台Yaw轴转角
* @param angle_y 输出,云台Pitch轴转角
* @param bullet_speed 输入,子弹初始速度
* @param current_ptz_angle 输入,云台Pitch轴当前角度
* @param offset 输入,rect 的偏移, 默认 (0,0)
*/
class AngleSolver : public RectPnPSolver
{
public:
    AngleSolver(const cv::Mat & cam_matrix, const cv::Mat & dist_coeff,
                double target_width = 0, double target_height = 0, double z_scale = 1.0,
                double min_dist = 50.0, double max_dist = 600.0)
        :RectPnPSolver(cam_matrix,dist_coeff,target_width,target_height)
    {
        min_distance = min_dist;
        max_distance = max_dist;
        scale_z = z_scale;
        //init_parameter
        trans_camera2ptz = cv::Mat::zeros(3,1,CV_64FC1);
        rot_camera2ptz = cv::Mat::eye(3,3,CV_64FC1);
    }
    double getAngle_Carmer(){
        double r11 = rot.ptr<double>(0)[0];
    //    double r12 = rotM.ptr<double>(0)[1];
    //    double r13 = rotM.ptr<double>(0)[2];
        double r21 = rot.ptr<double>(1)[0];
    //    double r22 = rotM.ptr<double>(1)[1];
    //    double r23 = rotM.ptr<double>(1)[2];
        double r31 = rot.ptr<double>(2)[0];
        double r32 = rot.ptr<double>(2)[1];
        double r33 = rot.ptr<double>(2)[2];
        double thetaz = atan2(r21, r11) / CV_PI * 180;
        double thetay = atan2(-1 * r31, sqrt(r32*r32 + r33*r33)) / CV_PI * 180;
        double thetax = atan2(r32, r33) / CV_PI * 180;
        std::cout << "相机的三轴旋转角：" <<"Pitch:"<< -1 * thetax << "  Roll: " << -1 * thetay <<"  Yaw: " << -1 * thetaz << std::endl;

    }

    void Init_RelationPtzCarmera(const cv::Mat &r_ptz_carmera,const cv::Mat &t_ptz_carmera, double offset_y_ptz_barrel);
    void changcamerToReal(const cv::Mat &pos, cv::Mat & transed_pos);      //将相机坐标系中坐标转化为云台坐标系
    void Barrel_angleRotation(const cv::Mat & pos_in_ptz, double & angle_x, double & angle_y, double bullet_speed = 0.0, double  current_angle = 0.0,const cv::Point2f & offset = cv::Point2f());
    bool getAngle_final(const cv::RotatedRect & rect, double & angle_x, double& angle_y,
                          double bullet_speed = 0,double current_ptz_angle = 0.0,const cv::Point2f &offset = cv::Point2f());        //返回坐标
public:
     cv::Mat position_in_camera;
     cv::Mat position_in_ptz;
private:
     cv::Mat rot;
    //检测距离
    double min_distance;
    double max_distance;
    double scale_z;
    //炮筒和云台再y轴的偏移量
    double offset_y_barrel_ptz;
    //
    cv::Mat trans_camera2ptz;       //平移矩阵
    cv::Mat rot_camera2ptz;         //旋转矩阵


};
class AngleFactory
{
public:
    AngleFactory() {}


    typedef enum { TARGET_ARMOR, TARGET_SAMLL_ATMOR} TargetType;
    void Init_SetArmorSize(double width, double height,TargetType type )
    {
        if(type == TARGET_ARMOR)
        {
            armor_height = height;
            armor_width = width;
        }
        if(type == TARGET_SAMLL_ATMOR)
        {
            armor_small_height = height;
            armor_small_width = width;
        }
    }
    void adjustRect(cv::RotatedRect &rect, double wh_ratio);
    void setSlover(AngleSolver * solver_){
        slover = solver_;
    }

    bool getAngle(const cv::RotatedRect &rect,TargetType type, double & angle_x, double & angle_y,
                  double bullet_speed, double current_ptz_angle, const cv::Point2f & offset);
private:
    double armor_width;
    double armor_height;
    double armor_small_width;
    double armor_small_height;

    AngleSolver * slover;
};
#endif // ANGLESOLVER_H

#include "Anglesolver.h"
#include "opencv2/opencv.hpp"

void RectPnPSolver::getTarget2dPoint(const cv::RotatedRect & rect,
                                   std::vector<cv::Point2f> & target2d)
{
    cv::Point2f vertices[4];
    rect.points(vertices);
    cv::Point2f lu,ld,ru,rd;        //right_down right_up left_up left_down
    std::sort(vertices,vertices + 4,[](const cv::Point2f &p1, const cv::Point2f &p2){ return p1.x < p2.x;});
    if(vertices[0].y < vertices[1].y)
    {
        lu = vertices[0];
        ld = vertices[1];
    }
    else {
        lu = vertices[1];
        ld = vertices[0];
    }
    if(vertices[2].y < vertices[3].y)
    {
        ru = vertices[2];
        rd = vertices[3];
    }
    else {
        ru = vertices[3];
        rd = vertices[2];
    }
    target2d.clear();
    target2d.push_back(lu);
    target2d.push_back(ru);
    target2d.push_back(rd);
    target2d.push_back(ld);
}

void RectPnPSolver::solvePnP4Point(const std::vector<cv::Point2f> &point2d, cv::Mat &rot, cv::Mat &trans)
{
    if( width_target < 10e-5||height_target < 10e-5){
        rot = cv::Mat::eye(3,3,CV_64FC1);
        trans = cv::Mat::zeros(3,1,CV_64FC1);
        return ;
    }
    std::vector<cv::Point3f> point3d;
    double half_x = width_target/2.0;
    double half_y = height_target/2.0;

    point3d.push_back(cv::Point3f(-half_x,-half_y,0));
    point3d.push_back(cv::Point3f(half_x,-half_y,0));
    point3d.push_back(cv::Point3f(half_x,half_y,0));
    point3d.push_back(cv::Point3f(-half_x,half_y,0));

    cv::Mat r;
    cv::solvePnP(point3d, point2d, camera_matrix, distortion_coeff, r, trans);
    cv::Rodrigues(r,rot);
}
void AngleSolver::Init_RelationPtzCarmera(const cv::Mat &r_ptz_carmera, const cv::Mat &t_ptz_carmera, double offset_y_ptz_barrel)
{
    r_ptz_carmera.copyTo(r_ptz_carmera);
    t_ptz_carmera.copyTo(trans_camera2ptz);
    offset_y_barrel_ptz = offset_y_ptz_barrel;
}

void AngleSolver::changcamerToReal( const cv::Mat &pos, cv::Mat & transed_pos)
{
    transed_pos = rot_camera2ptz * pos + trans_camera2ptz;
}
//void AngleSolver::Barrel_angleRotation(const cv::Mat &pos_in_ptz, double &angle_x, double &angle_y, double bullet_speed, double current_angle, const cv::Point2f &offset)
//{
//    const double *_xyz = (const double *)pos_in_ptz.data;
//    double down_t = 0;
//    if(bullet_speed > 10e-3)
//        down_t = _xyz[2]/100/bullet_speed;
//    double offset_gravity_y = 0.5 * 9.8 * down_t*down_t *100;
//    double xyz[3] = { _xyz[0] , _xyz[1] - offset_gravity_y , _xyz[2] };
//    double alpha = 0.0,theta = 0.0;
//    alpha = asin(offset_y_barrel_ptz / (xyz[1] * xyz[1] + xyz[2] * xyz[2]));
//    if(xyz[1] < 0)
//    {
//        theta = atan(-xyz[1] / xyz[2]);
//        angle_y = -(alpha + theta);
//    }
//    else if(xyz[1] < offset_y_barrel_ptz){
//        theta = atan(xyz[1]/xyz[2]);
//        angle_y = -(alpha - theta);
//    }
//    else{
//        theta = atan(xyz[1]/xyz[2]);
//        angle_y = (theta - alpha);
//    }
//    angle_x = atan2(xyz[0], xyz[2]);


//    angle_x = angle_x *180 / 3.1415926;
//    angle_y = angle_y *180 / 3.1415926;

//}
void AngleSolver::Barrel_angleRotation(const cv::Mat &pos_in_ptz, double &angle_x, double &angle_y, double bullet_speed, double current_angle, const cv::Point2f &offset)
{
    const double * xyz = (const double *) pos_in_ptz.data;
    angle_x = atan2(xyz[0] ,xyz[2]);
    angle_y = atan(xyz[1]/xyz[2]);
}

bool AngleSolver::getAngle_final(const cv::RotatedRect &rect, double &angle_x, double &angle_y, double bullet_speed, double current_ptz_angle, const cv::Point2f &offset)
{
    if(rect.size.height < 1)
        return false;

    std::vector<cv::Point2f> target2d;
    getTarget2dPoint(rect, target2d);
    solvePnP4Point(target2d, rot, position_in_camera);
    changcamerToReal(position_in_camera, position_in_ptz);
    std::cout<<position_in_camera<<std::endl;
    Barrel_angleRotation(position_in_ptz, angle_x, angle_y, bullet_speed, current_ptz_angle, offset);

    return true;
}

//
void AngleFactory::adjustRect(cv::RotatedRect &rect, double wh_ratio){
    rect.size.height = rect.size.width / wh_ratio;
}

bool AngleFactory::getAngle(const cv::RotatedRect &rect, TargetType type, double &angle_x, double &angle_y, double bullet_speed, double current_ptz_angle, const cv::Point2f &offset)
{
    if(slover == NULL){
        std::cerr << "slover not set\n";
        return false;
    }
    double width =0.0, height = 0.0;
    if(type == TARGET_ARMOR){
        width = armor_width;
        height = armor_height;
    }
    else if (type == TARGET_SAMLL_ATMOR) {
        width = armor_small_width;
        height = armor_small_height;
    }
    cv::RotatedRect rect_rectifid = rect;
    AngleFactory::adjustRect(rect_rectifid, width/height);
    slover->setTargetSize(width, height);
    return slover->getAngle_final(rect_rectifid, angle_x, angle_y, bullet_speed, current_ptz_angle, offset);
}



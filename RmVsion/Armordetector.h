#pragma once
#include "opencv2/highgui/highgui.hpp"
#include "Anglesolver.h"
using namespace cv;
using namespace std;
    enum EnemyColor { RED = 0, BLUE = 1 };
    struct ArmorParam {
        uchar min_light_gray;	        // 板灯最小灰度值
        uchar min_light_height;			// 板灯最小高度值
        uchar avg_contrast_threshold;	// 对比度检测中平均灰度差阈值，大于该阈值则为显著点
        uchar light_slope_offset;		// 允许灯柱偏离垂直线的最大偏移量，单位度
        uchar  max_light_delta_h;         // 左右灯柱在水平位置上的最大差值，像素单位
        uchar min_light_delta_h;		// 左右灯柱在水平位置上的最小差值，像素单位
        uchar max_light_delta_v;		// 左右灯柱在垂直位置上的最大差值，像素单位
        uchar max_light_delta_angle;	// 左右灯柱在斜率最大差值，单位度
        uchar avg_board_gray_threshold; // 矩形区域平均灰度阈值，小于该阈值则选择梯度最小的矩阵
        uchar avg_board_grad_threshold; // 矩形区域平均梯度阈值，小于该阈值则选择梯度最小的矩阵
        uchar grad_threshold;			// 矩形区域梯度阈值，在多个矩形区域中选择大于该阈值像素个数最少的区域  (not used)
        uchar br_threshold;				// 红蓝通道相减后的阈值
        uchar enemy_color;                 // 0 for red, otherwise blue

        ArmorParam() {
            min_light_gray = 210;
            min_light_height = 8;
            avg_contrast_threshold = 80;
            light_slope_offset = 30;
            max_light_delta_h = 450;
            min_light_delta_h = 12;
            max_light_delta_v = 50;
            max_light_delta_angle = 30;
            avg_board_gray_threshold = 80;
            avg_board_grad_threshold = 25;
            grad_threshold = 25;
            br_threshold = 30;
            enemy_color = 0;
        }
    };





class ArmorDetector
{
public:
    RotatedRect getArmor_Final(const cv::Mat & src);
    //ArmorDetector(const ArmorParam & para = ArmorParam())
    //{
    //	pitch_angle = 0;
    //	s_solver = NULL;
    //	l_solver = NULL;
    //	_para = para;
    //	_res_last = cv::RotatedRect();
    //	_dect_rect = cv::Rect();
    //	_is_small_armor = false;
    //	_lost_cnt = 0;
    //	_is_lost = true;
    //}
    /*~ArmorDetector();*/
#define POINT_DIST(p1,p2) std::sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y))

    void initTemplate(const Mat &_template, const Mat &_template_small);
    /*RotatedRect getTargetAera(const Mat &src);*/
    bool Armor_isSmall()
    {
        return _is_small_armor;
    }

private:
    /***********判断有无上次矩形，有则处理上次区域，进行二值化、分离通道*******************/


    void setImage(const cv::Mat & src);
    /*************************采用模板匹配、找到左右两个模板*******************************/


    void findContourInEnemyColor(
        cv::Mat & left, cv::Mat & right,
        vector<vector<Point2i> > &contours_left,
        vector<vector<Point2i> > &contours_right);
    /*************************************寻找可疑矩形************************************/


    cv::RotatedRect boundingRRect(const cv::RotatedRect & left, const cv::RotatedRect & right);
    RotatedRect adjustRRect(const RotatedRect & rect);
    void findTargetInContours(const vector<vector<Point> > & contours_left, const vector<vector<Point> > & contours_right, vector<RotatedRect> & rects, std::vector<double> & score);
    /*************************************选择最终矩形************************************/

    bool broadenRect(cv::Rect & rect, int width_added, int height_added, cv::Size size);
    bool makeRectSafe(cv::Rect & rect, cv::Size size) {
        if (rect.x < 0)
            rect.x = 0;
        if (rect.x + rect.width > size.width)
            rect.width = size.width - rect.x;
        if (rect.y < 0)
            rect.y = 0;
        if (rect.y + rect.height > size.height)
            rect.height = size.height - rect.y;
        if (rect.width <= 0 || rect.height <= 0)
            return false;
        return true;
    }
    int templateDist(const cv::Mat & img, bool is_small);
    RotatedRect chooseTarget(const std::vector<cv::RotatedRect> & rects, const std::vector<double> & score);
private:
    AngleSolver * s_solver;

    //double pitch_angle;
    bool _is_lost;
    int  _lost_cnt;
    bool _is_small_armor;
    RotatedRect _res_last;
    Rect _dect_rect;
    ArmorParam _para;
    Mat _binary_template;
    Mat _binary_template_small;
    Mat _src;
    Mat _g;
    Mat _ec;
    Mat _max_color;
    Size _size;

};

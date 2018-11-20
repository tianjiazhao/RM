#include "Predictor.h"
#include "opencv2/opencv.hpp"
#include <algorithm>

bool Predictor::setRecord(double value, double time)
{
    if(history_value.size() <5 ){
        history_value.push_back(value);
        history_time.push_back(time);
    }
    else{
        history_value.push_back(value);
        history_time.push_back(time);
        history_value.pop_front();
        history_value.pop_front();
    }
}

double Predictor::predictor(double time)
{
    std::list<double >::const_iterator it_in = history_time.begin();
    double last_value = history_value.back();
    if(abs(last_value)< 5 || history_value.size() <history_size){
        return last_value;
    }
    if(history_time.back() - *it_in >150.0){
        return last_value;
    }
    std::list<double>::const_iterator it_out = history_value.begin();
    std::list<double>::const_iterator previous_out = it_out;
    double max_o = -50000,min_o = 50000;
    for(std::list<double>::const_iterator it = it_out,it_i = it_in; it !=history_value.end(); ++it,++it_i){
        if(max_o < *it)
            max_o = *it;
        if(min_o > *it)
            min_o = *it;
        if(abs(*previous_out - *it)> 5){
            return last_value;              //几次的差距太大。
        }
        previous_out = it;
    }

   if(max_o - min_o >3)     //角度的差距不能大于3度
       return last_value;
   cv::Mat A(history_size,3 ,CV_64F);
   cv::Mat b(history_size,1 ,CV_64F);
   double * b_data = (double *)b.data;
   double * A_data = (double *)A.data;
   for(;it_in != history_time.end(); ++b_data,++A_data,++it_in,++it_out)
   {
       *b_data = * it_out;
       *A_data = (*it_in - time)*(*it_in -time);
       *(++A_data) = (*it_in -time);
       *(++A_data) = 1;
   }
   //A*w = b => w = A.inverse *b
   cv::Mat A_t = A.t();
   cv::Mat w = (A_t *A).inv() *A_t *b;
   cv::Mat q = (cv::Mat_<double > (1,3) <<0 ,0, 1);
   cv::Mat ret = q*w;
   double predictor_angle = ret.at<double>(0);
   const double max_gap = 3.0;
   if(predictor_angle - last_value >max_gap)
       predictor_angle = predictor_angle -last_value;
   else if(predictor_angle - last_value < -max_gap)
       predictor_angle = last_value - max_gap;
   return predictor_angle;



}

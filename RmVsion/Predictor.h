#ifndef PREDICTOR_H
#define PREDICTOR_H
#include <iostream>
#include <list>

class Predictor
{
public:
    Predictor(int size_history=5){
        history_size = size_history;
    }

    double predictor(double time);
    bool setRecord(double value, double time);
private:
    std::list<double>  history_value;
    std::list<double>  history_time;
    int history_size;
};

#endif // PREDICTOR_H

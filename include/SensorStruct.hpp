#pragma once
#include "Eigen/Dense"
#include <vector>
#include<iostream>


enum class SENSOR { SPEDOMETER, ACCEL };

struct SensorStruct
{
    double refresh_rate;
    Eigen::VectorXd measurement;
    Eigen::MatrixXd H;
    Eigen::VectorXd R;
    SensorStruct(SENSOR sensor_type)
    {
        switch (sensor_type)
        {
            case SENSOR::SPEDOMETER :
                H.resize(1,6); 
                H << 0,0,1,0,0,0;
                R.resize(1); R << 5;
                break;
            case SENSOR::ACCEL :
                H.resize(1,6); 
                H << 0,0,0,1,0,0;
                R.resize(1); R << 10;
                break;
            default :
                std::cerr << "unknown sensor type";
                break;
        }
    }
    
};

typedef std::vector < SensorStruct >    SensorSet;

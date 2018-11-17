#pragma once
#include "Eigen/Dense"
#include <vector>
#include<iostream>


enum class SENSOR { SPEDOMETER, IMU };

struct SensorStruct
{
    double refresh_rate;
    Eigen::MatrixXd measurement;
    Eigen::Vector3d H;
    SensorStruct(SENSOR sensor_type)
    {
        switch (sensor_type)
        {
            case SENSOR::SPEDOMETER : 
                H = Eigen::Vector3d(0,1,0);
                break;
            case SENSOR::IMU :
                H = Eigen::Vector3d(0,0,1);
                break;
            default :
                std::cerr << "unknown sensor type";
                break;
        }
    }
    
};

struct SensorStack
{
    SensorStruct imu = SensorStruct(SENSOR::IMU);
    SensorStruct spedometer = SensorStruct(SENSOR::SPEDOMETER);
};

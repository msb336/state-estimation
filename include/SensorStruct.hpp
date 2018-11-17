#include "Eigen/Dense"


struct Spedometer
{
    double_refresh_rate;
    Eigen::Matrix<double,1,1> measurement;
    Eigen::Vector3d H = Eigen::Vector3d(0,1,0);
    
};


struct IMU
{
    double refresh_rate;
    Eigen::Matrix<double,1,1> acceleration;
    Eigen::Vector3d H = Eigen::Vector3d(0,0,1);
};

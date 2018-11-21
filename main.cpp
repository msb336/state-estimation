// #include "KalmanFilter.hpp"
// #include "SensorStruct.hpp"
#include "SystemDynamics.hpp"
#include "common.h"
#include "TwoDimBot.hpp"
#include "Robot.hpp"
#include "KalmanFilter.hpp"


int main ()
{
    srand(time(0));

    // Filewrite setup
    std::ofstream fileStream;
    fileStream.open("position.csv", std::ios::out);
    fileStream << "time,kx,ky,kv,ka,ktheta,komega,u1,u2\n";//,x,y,v,a,theta,omega,omega_reading,spedometer\n";
    Eigen::VectorXd u;
    double dt = 0.01;

    SensorSet sensors; sensors.push_back(SensorStruct(SENSOR::ACCEL)); sensors.push_back(SENSOR::SPEDOMETER);
    TwoDimBot robot_model(dt);

    SystemDynamics true_system(robot_model, Eigen::VectorXd::Zero(6));
    KalmanFilter kf(robot_model, sensors, Eigen::VectorXd::Zero(6), dt);
    

    for (int t=0; t<1000; t++)
    {
        if (t < 200 )
        {u = Eigen::Vector2d(0.5,0);}
        else if ( t < 400)
        {u = Eigen::Vector2d(0,1);}
        else
        {u = Eigen::Vector2d(0,1);}

        auto position = true_system.getState(u);
        Eigen::Vector2d measurement(position(3)*(1+frand()), position(5)*(1+frand()));
        auto kfpos = kf.filter(u, measurement);

        write(&fileStream, t, kfpos, u);
    }

    return 0;
}
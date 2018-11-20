// #include "KalmanFilter.hpp"
// #include "SensorStruct.hpp"
#include "SystemDynamics.hpp"
#include "common.h"
#include "TwoDimBot.hpp"
#include "Robot.hpp"


int main ()
{
    // Filewrite setup
    std::ofstream fileStream;
    fileStream.open("position.csv", std::ios::out);
    fileStream << "time,x,y,v,theta,u1,u2\n";

    Eigen::VectorXd u;
    double dt = 0.01;
    TwoDimBot robot_model(dt);

    SystemDynamics true_system(robot_model, Eigen::VectorXd::Zero(4));

    for (int t=0; t<1000; t++)
    {
        if (t < 200 )
        {u = Eigen::Vector2d(0.5,0);}
        else if ( t < 400)
        {u = Eigen::Vector2d(0,1);}
        else
        {u = Eigen::Vector2d(0,1);}

        auto position = true_system.getState(u);
        write(&fileStream, t, position, u);
    }

    return 0;
}
// #include "KalmanFilter.hpp"
// #include "SensorStruct.hpp"
#include "SystemDynamics.hpp"
#include<fstream>
#include <time.h>
#include<iostream>


double frand()
{
    return 2*((double)rand() / RAND_MAX-0.5);
}


int main ()
{
    // Filewrite setup
    std::ofstream fileStream;
    fileStream.open("position.csv", std::ios::out);
    fileStream << "time,inputx,inputy,inputz,x,dx,y,dy,z,dz\n";

    // // Sensor setup
    // srand(time(0));



    double dt = 0.01;

    SystemDynamics true_system(Eigen::MatrixXd::Zero(6,1), dt);
    Eigen::Vector3d u;
    for (int t=0; t<1000; t++)
    {
        if (t < 100 )
        {u = Eigen::Vector3d(0.5,0,0);}
        else if ( t < 300)
        {u = Eigen::Vector3d(0,0.5,0);}
        else
        {u = Eigen::Vector3d(-5,0,0.5);}

        auto position = true_system.getState(u);

        fileStream << t << "," << u(0) << "," << u(1) << "," << u(2) << "," << 
        position(0) << "," << position(1) <<  "," << 
        position(2) << "," << position(3) << "," << 
        position(4) << "," << position(5) << "," << "\n";
        
        
    }


    return 0;
}
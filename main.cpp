#include "include/KalmanFilter.hpp"
#include<fstream>
#include <time.h>
#include<iostream>
double frand()
{
    return 2*((double)rand() / RAND_MAX-0.5);
}

int main ()
{
    srand(time(0));
    std::ofstream fileStream;
    fileStream.open("position.csv", std::ios::out);
    fileStream << "time,input,measured velocity,measured acceleration,true pos, true vel, true accel,est pos, est vel, est accel\n";
    double dt = 0.01;
    KalmanFilter kf(Eigen::Vector3d(0,0,0),dt), kf_truth(Eigen::Vector3d(0,0,0),dt);
    double u = 0.5;
    Eigen::Vector2d measure(0,0);
    Eigen::Vector3d position(0,0,0);
    Eigen::Vector3d truth;
    for (int t=0; t<1000; t++)
    {
        if (t < 100 )
        {u = 0.5;}
        else if ( t < 300)
        {u = 0;}
        else
        {u = -0.5;}

        position = kf.filter(u,measure);
        truth = kf_truth.justState(u);

        measure << truth(1)*(1 + frand()*0.1), truth(2)*(1 + frand()*0.1);

        fileStream << t << "," << u << "," << measure(0) << "," << measure(1) << "," <<
        truth(0) << "," << truth(1) << "," << truth(2) <<  "," << 
        position(0) << "," << position(1) <<  "," << position(2) << "\n";
        
        
    }


    return 0;
}
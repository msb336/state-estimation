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
    fileStream << "time,input,vel reading, true pos,true vel,estimated pos, estimated vel, vel pos input\n";
    double dt = 0.01;
    KalmanFilter kf(Eigen::Vector2d(0,0),dt), kf_truth(Eigen::Vector2d(0,0),dt);
    double u = 0.5;
    double v = 0;
    Eigen::Vector2d position(0,0);
    Eigen::Vector2d truth;
    for (int t=0; t<1000; t++)
    {
        if (t < 100 )
        {u = 0.5;}
        else if ( t < 300)
        {u = 0;}
        else
        {u = -0.5;}

        position = kf.filter(u,v*dt+position(0));
        truth = kf_truth.justState(u);
        v = truth(1)*(1 + frand()*0.1);
        std::cout << truth(0) << " " << truth(1) <<  " ....  " << position(0) << " " << position(1) << std::endl;
        fileStream << t << "," << u << "," << v << "," 
        << truth(0) << "," << truth(1) 
        <<  "," << position(0) << "," << position(1) 
        << "," << v*dt + position(0) << "\n";
        
    }


    return 0;
}

#pragma once
#include<fstream>
#include <time.h>
#include<iostream>
#include "Eigen/Dense"

double frand()
{
    return 2*((double)rand() / RAND_MAX-0.5);
}

void setStateSpace( Eigen::MatrixXd& A, Eigen::MatrixXd& B, const double& dt)
{
    A.resize(9,9); B.resize(9,3);
    
    A.fill(0);                  B.fill(0);
    A(0,0) = 1; A(0,1) = dt;
    A(1,1) = 1; A(1,2) = dt;
                                B(2,0) = 1;
    A(3,0) = 1; A(0,1) = dt;
    A(4,1) = 1; A(1,2) = dt;
                                B(5,1) = 1;
    A(6,0) = 1; A(0,1) = dt;
    A(7,1) = 1; A(1,2) = dt;
                                B(8,2) = 1;
    
    
}


void write(std::ofstream* filestream, const int& time, Eigen::VectorXd state, Eigen::VectorXd input)
{
    *filestream << time;
    for(int i=0; i<state.size(); i++)
    {
        *filestream << "," << state(i);
    }
    for(int j=0; j<input.size(); j++)
    {
        *filestream << "," << input(j);
    }
    *filestream << "\n";
}

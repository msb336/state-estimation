#pragma once
#include "Eigen/Dense"
#include<iostream>

class Robot
{
private:
    /* data */
public:
    Robot(/* args */);
    ~Robot();
    Eigen::MatrixXd A_;
    Eigen::MatrixXd B_;
    Eigen::MatrixXd Q_;
    virtual void setMatrices(const double&, const Eigen::VectorXd& ){};
    virtual void setMatrices(const Eigen::VectorXd& ){std::cout<<"in the virtual function" << std::endl;};
};



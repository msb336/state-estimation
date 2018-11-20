#include "Eigen/Dense"
#pragma once
#include "cmath"
#include "Robot.hpp"
#include <iostream>

class TwoDimBot : public Robot
{
public:
    void setMatrices(const double&, const Eigen::VectorXd& );
    void setMatrices(const Eigen::VectorXd& );
public:

    double dt_;
    TwoDimBot( const double& );
    ~TwoDimBot();

};

TwoDimBot::TwoDimBot(const double& dt)
{
    dt_ = dt;
    setMatrices(dt, Eigen::VectorXd::Zero(6) );
}

TwoDimBot::~TwoDimBot()
{
}

void TwoDimBot::setMatrices(const double& dt, const Eigen::VectorXd& state)
{
    A_.resize(4,4);
    B_.resize(4,2);

    A_(0,0) = 1;
    A_(1,1) = 1;
    A_(2,2) = 1;
    A_(3,3) = 1;

    setMatrices(state);
}
void TwoDimBot::setMatrices(const Eigen::VectorXd& state)
{
    A_(0,2) = cos(state(3))*dt_;
    A_(1,2) = sin(state(3))*dt_;

    B_(2,0) = 1;
    B_(3,1) = dt_;
}
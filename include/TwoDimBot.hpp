#pragma once
#include "Eigen/Dense"
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
    double sigma_ = 0.25;
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

    //States: x,y,v, a, theta about z axis, omega about z axis
    
    A_.resize(6,6);
    B_.resize(6,2);
    Q_.resize(6,6);
    Eigen::Matrix<double,6,1> G;

    G << 0.5*dt*dt, 0.5*dt*dt, dt, dt, dt, dt;

    A_(0,0) = 1;
    A_(1,1) = 1;
    A_(2,2) = 1;    A_(2,3) = dt;
    A_(4,4)  = 1;    A_(4,5) = dt;

    Q_ = G*G.transpose()*sigma_;

    setMatrices(state);
}
void TwoDimBot::setMatrices(const Eigen::VectorXd& state)
{
    A_(0,2) = cos(state(4))*dt_;
    A_(1,2) = sin(state(4))*dt_;

    B_(3,0) = 1;
    B_(5,1) = 1;
}
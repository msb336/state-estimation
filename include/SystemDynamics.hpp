#pragma once
#include "Eigen/Dense"
#include "Robot.hpp"

class SystemDynamics
{
public:
    SystemDynamics( Robot&, const Eigen::VectorXd&);
    ~SystemDynamics();

    Eigen::VectorXd getState(const Eigen::MatrixXd& );
private:
    Robot* robot_;
    Eigen::VectorXd state_;
};
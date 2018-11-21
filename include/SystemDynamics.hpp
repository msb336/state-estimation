#include "Eigen/Dense"
#include "Robot.hpp"
#include <iostream>

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

SystemDynamics::SystemDynamics( Robot& robot_type, const Eigen::VectorXd& start )
{
    robot_ = &robot_type;   state_=start; std::cout << "built SystemDynamics object"<<std::endl;}
SystemDynamics::~SystemDynamics()
{
}

Eigen::VectorXd SystemDynamics::getState(const Eigen::MatrixXd& u)
{
    robot_->setMatrices ( state_ );
    state_ = robot_->A_*state_ + robot_->B_*u;
    return state_;
}
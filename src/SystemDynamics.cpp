#include "SystemDynamics.hpp"

SystemDynamics::SystemDynamics( Robot& robot_type, const Eigen::VectorXd& start )
{
    robot_ = &robot_type;   state_=start; }
SystemDynamics::~SystemDynamics()
{
}

Eigen::VectorXd SystemDynamics::getState(const Eigen::MatrixXd& u)
{
    robot_->setMatrices ( state_ );
    state_ = robot_->A_*state_ + robot_->B_*u;
    return state_;
}
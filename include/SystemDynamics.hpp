#include "Eigen/Dense"

class SystemDynamics
{
public:
    typedef Eigen::Vector3d Vector3d;

    SystemDynamics(const Eigen::Matrix<double,6,1>&, const double& );
    ~SystemDynamics();
    Eigen::Matrix<double,6,1> getState(const Eigen::Matrix<double,3,1>& );
private:
    Eigen::Matrix<double, 6, 6> A_;
    Eigen::Matrix<double, 6, 3> B_;
    Eigen::Matrix<double, 6,1> state_;

};

SystemDynamics::SystemDynamics(const Eigen::Matrix<double,6,1>& start, const double& dt)
{
    A_ <<   1,  dt,  0,  0,  0,  0,  
            0,  1,  0,  0,  0,  0, 
            0,  0,  1,  dt, 0,  0,
            0,  0,  0,  1,  0,  0,
            0,  0,  0,  0,  1,  dt,
            0,  0,  0,  0,  0,  1;
    B_ <<   0, 0,  0,
            dt, 0,  0,
            0,  0,  0,
            0,  dt, 0,
            0,  0,  0,
            0,  0,  dt;  
    state_ = start;
}

SystemDynamics::~SystemDynamics()
{
}

Eigen::Matrix<double,6,1> SystemDynamics::getState(const Eigen::Matrix<double,3,1>& u)
{
    state_ = A_*state_ + B_*u;
    return state_;
}
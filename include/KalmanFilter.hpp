#include "Eigen/Dense"
#include<iostream>
#include "SensorStruct.hpp"
class KalmanFilter
{
public:
    typedef Eigen::Vector3d Vector3d;

    KalmanFilter(const Vector3d&, const double& );
    ~KalmanFilter();
    Vector3d predictState(const double& );
    Vector3d justState(const double& );
    Vector3d filter(const double&, const Eigen::Vector2d& );

private:
    Eigen::Matrix<double, 3, 3> A_;
    Eigen::Matrix<double, 3, 3> P_;
    Eigen::Matrix<double, 3, 3> Q_;
    Eigen::Matrix<double, 2, 3> H_;
    Eigen::Matrix<double, 2, 2> R_;
    Eigen::Matrix<double, 3, 2> K_;

    Vector3d B_;
    Vector3d state_;
    Vector3d predicted_state_;
    Eigen::Vector2d measurement_;

    void predictCovariance();
    void innovate(const Eigen::Vector2d&);
    void updateStateEstimate();
    void updateCovariance();
};


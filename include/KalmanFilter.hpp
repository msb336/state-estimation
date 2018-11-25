#pragma once
#include "Eigen/Dense"
#include "SensorStruct.hpp"
#include "Robot.hpp"

class KalmanFilter
{
public:
    typedef Eigen::VectorXd Vector;
    KalmanFilter(Robot&, const SensorSet&, const Vector&, const double& );
    ~KalmanFilter();
    Vector predictState(const Vector& );
    Vector justState(const double& );
    Vector filter(const Vector&, const Vector& );

private:
    Robot* model_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd H_;
    Eigen::MatrixXd K_;
    Vector R_;

    Vector state_;
    Vector predicted_state_;
    Vector measurement_;

    void predictCovariance();
    void innovate(const Vector&);
    void updateStateEstimate();
    void updateCovariance();
    void assembleFromSensors(const SensorSet&);
    void addSensor( const SensorStruct& );
};
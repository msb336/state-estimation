#include "Eigen/Dense"
#include<iostream>
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

KalmanFilter::KalmanFilter(Robot& robot, const SensorSet& sensors, const Eigen::VectorXd& state, const double& dt =0.01  )
{
    model_ = &robot;
    double sigma = 0.25;
    state_ = state;
    predicted_state_ = state;
    assembleFromSensors(sensors);
    P_.resize(model_->A_.rows(), model_->A_.cols());
}

KalmanFilter::~KalmanFilter()
{
}
void KalmanFilter::assembleFromSensors(const SensorSet& sensors)
{
    for(auto it = sensors.begin(); it != sensors.end(); it++)
    {
        addSensor(*it);
    }
}

void KalmanFilter::addSensor(const SensorStruct& sensor)
{
    Eigen::MatrixXd H_temp = H_;
    Eigen::VectorXd R_temp = R_;
    H_.resize(H_.rows()+sensor.H.rows(), sensor.H.cols());
    int new_rows = R_.rows()+sensor.R.rows();
    R_.resize(new_rows);

    if (H_temp.size() != 0 )
    {
        H_ << H_temp, sensor.H;
        R_ << R_temp, sensor.R;
    }
    else
    {
        H_ << sensor.H;
        R_ << sensor.R;
    }

}

Eigen::VectorXd KalmanFilter::predictState(const Eigen::VectorXd& u)
{
    model_->setMatrices(state_);
    predicted_state_ = model_->A_*state_ + model_->B_*u;
    return predicted_state_;
}

void KalmanFilter::predictCovariance()
{
    P_ = model_->A_*P_*model_->A_.transpose() + model_->Q_;
}

void KalmanFilter::innovate(const Eigen::VectorXd&  sensor_reading)
{
    measurement_ = sensor_reading - H_*predicted_state_;
    Eigen::MatrixXd R = R_.asDiagonal();
    Eigen::MatrixXd S = H_*P_*H_.transpose() + R;
    K_ = P_*H_.transpose()*S;
}

void KalmanFilter::updateStateEstimate()
{
    state_ = predicted_state_ + K_ *measurement_;
    P_ = P_ - (K_*H_*P_);
}

Eigen::VectorXd KalmanFilter::filter(const Eigen::VectorXd& u, const Eigen::VectorXd& measurement)
{
    predictState(u);
    predictCovariance();
    innovate(measurement);
    updateStateEstimate();
    return state_;
}
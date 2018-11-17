#include "Eigen/Dense"
#include<iostream>
#include "SensorStructs.hpp"
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

KalmanFilter::KalmanFilter(const Eigen::Vector3d& state = Eigen::Vector3d(0,0,0), const double& dt =0.01  )
{
    double sigma = 0.25;
    state_ = state;
    predicted_state_ = state;
    A_ <<   1,  dt, 0,
            0,  1,  dt,
            0,  0,  0;
    B_ <<   0,  0,  1;
    P_ <<   0.1,    0,  0,
            0,  0,  0,
            0,  0,  0.1;

    Eigen::Matrix<double,3,2> G;
    
    G << 0.5*dt*dt,   0.5*dt*dt,  
            dt, dt,
            0,  0 ;
                                
    Q_ = G*G.transpose()*sigma;
    H_ <<   0,  1,  0,
            0,  0,  1;
    R_ << 10, 10, 10, 10;

}

KalmanFilter::~KalmanFilter()
{
}

Eigen::Vector3d KalmanFilter::predictState(const double& u)
{
    predicted_state_ = A_*state_ + B_*u;
    return predicted_state_;
}
Eigen::Vector3d KalmanFilter::justState(const double& u)
{
    predicted_state_ = A_*predicted_state_ + B_*u;
    return predicted_state_;
}

void KalmanFilter::predictCovariance()
{
    P_ = A_*P_*A_.transpose() + Q_;
}

void KalmanFilter::innovate(const Eigen::Vector2d&  sensor_reading)
{
    measurement_ = sensor_reading - H_*predicted_state_;
    Eigen::Matrix<double,2,2> S = H_*P_*H_.transpose() + R_;
    K_ = P_*H_.transpose()*S;
}

void KalmanFilter::updateStateEstimate()
{
    state_ = predicted_state_ + K_ *measurement_;
    P_ = P_ - (K_*H_*P_);
}

Eigen::Vector3d KalmanFilter::filter(const double& u, const Eigen::Vector2d& measurement)
{
    predictState(u);
    // std::cout << "predicted state: " << predicted_state_(0) << " " << predicted_state_(1) << std::endl;
    predictCovariance();
    innovate(measurement);
    updateStateEstimate();
    // std::cout << "state estimate: " << state_(0) << " " << state_(1) << std::endl;
    return state_;
}
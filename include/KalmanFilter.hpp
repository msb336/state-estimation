#include "Eigen/Dense"
#include<iostream>

class KalmanFilter
{


public:
    typedef Eigen::Vector3d Vector3d;
    typedef Eigen::Vector2d Vector2d;
    KalmanFilter(const Vector2d&, const double& );
    ~KalmanFilter();
    Vector2d predictState(const double& );
    Vector2d justState(const double& );
    Vector2d filter(const double&, const double& );

private:
    Eigen::Matrix<double, 2, 2> A_;
    Eigen::Matrix<double, 2, 2> P_;
    Eigen::Matrix<double, 2, 2> Q_;
    Eigen::Matrix<double, 1, 2> H_;
    double R_;
    Eigen::Matrix<double, 2, 1> K_;

    Vector2d B_;
    Vector2d state_;
    Vector2d predicted_state_;
    double measurement_;

    void predictCovariance();
    void innovate(const double&);
    void updateStateEstimate();
    void updateCovariance();

};

KalmanFilter::KalmanFilter(const Eigen::Vector2d& state = Eigen::Vector2d(0,0), const double& dt =0.01  )
{
    double sigma = 0.25;
    state_ = state;
    predicted_state_ = state;
    A_ << 1,dt,0,1;
    B_ << 0,dt;
    P_ << 0.1,0,0,0.1;
    Eigen::Matrix<double,2,1> G;
    G << 0.5*dt*dt, dt;
    Q_ = G*G.transpose()*sigma;
    H_ << 1, dt;
    R_ = 10;

}

KalmanFilter::~KalmanFilter()
{
}

Eigen::Vector2d KalmanFilter::predictState(const double& u)
{
    predicted_state_ = A_*state_ + B_*u;
    return predicted_state_;
}
Eigen::Vector2d KalmanFilter::justState(const double& u)
{
    predicted_state_ = A_*predicted_state_ + B_*u;
    return predicted_state_;
}

void KalmanFilter::predictCovariance()
{
    P_ = A_*P_*A_.transpose() + Q_;
}

void KalmanFilter::innovate(const double& velocity_measure)
{
    measurement_ = velocity_measure - H_*predicted_state_;
    double S = H_*P_*H_.transpose() + R_;
    K_ = P_*H_.transpose()*S;
}

void KalmanFilter::updateStateEstimate()
{
    state_ = predicted_state_ + K_ *measurement_;
    P_ = P_ - (K_*H_*P_);
}

Eigen::Vector2d KalmanFilter::filter(const double& u, const double& measurement)
{
    predictState(u);
    // std::cout << "predicted state: " << predicted_state_(0) << " " << predicted_state_(1) << std::endl;
    predictCovariance();
    innovate(measurement);
    updateStateEstimate();
    // std::cout << "state estimate: " << state_(0) << " " << state_(1) << std::endl;
    return state_;
}
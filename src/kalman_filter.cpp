#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() = default;

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}


void KalmanFilter::Update_F_and_Q(float delta_T, int noise_x, int noise_y)
{
	F_(0, 2) = delta_T;
	F_(1, 3) = delta_T;
	
	float cov_1 = delta_T * delta_T ;
	float cov_2 = (cov_1 * cov_1) / 4.0;
	float cov_3 = (cov_1 * delta_T) / 2.0;
	
	Q_ << cov_2 * noise_x, 0.0,          cov_3 * noise_x, 0.0,
				0.0,           cov_2 * noise_y, 0.0, cov_3 * noise_y,
				cov_3 * noise_x, 0.0,          cov_1 * noise_x, 0.0,
				0.0,           cov_3 * noise_y, 0.0, cov_1 * noise_y;
}
void KalmanFilter::Predict() {
  		
	
	x_ = F_ * x_;
	P_ = F_ * P_ * F_.transpose() + Q_;
	
}

void KalmanFilter::Update(const VectorXd &z) {
  
	// error 
	MatrixXd y = z - (H_ * x_);
	KalmanMeasurement(y);
	
	
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  
	
	
	float px = x_(0);
	float py = x_(1);
	float vx = x_(2);
	float vy = x_(3);
	
	float rho = sqrt(px*px + py*py);
	float phi = atan2(py, px);
	
	// avoid zero division 
	if (rho < 0.001) 
	{
		rho = 0.001;
	}
	float rho_rate = (px*vx + py*vy) / rho;
	
	VectorXd h(z.rows());
	h << rho, phi, rho_rate;
	
	
	VectorXd y = z - h;
	
	// phi to be [-pi, pi] Learned it from AI for robotics
	static const double PI = 3.1415926;
	while (y(1) < -PI)
	{		
    y(1) += 2 * PI;
	}
  while (y(1) > PI)
	{		
    y(1) -= 2 * PI;
	}
	
	
	// process measurement
	KalmanMeasurement(y);
	
}

/*
 * Code Factorization 
 */

void KalmanFilter::KalmanMeasurement(const Eigen::VectorXd& y)
{
	
	MatrixXd S = H_ * P_ * H_.transpose() + R_;
	// kalman gain
	MatrixXd K = P_ * H_.transpose() * S.inverse();
		
	// correct state prediction
	x_ = x_ + K * y;
	static const MatrixXd I  = MatrixXd::Identity(P_.rows(), P_.rows());
	P_ = (I - (K * H_)) * P_;
	
}

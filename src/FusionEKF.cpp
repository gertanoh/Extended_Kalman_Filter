#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF(): ekf_() 
{
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
	
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0.0,
							0.0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0.0, 0.0,
							0.0, 0.009, 0.0,
							0.0, 0.0, 0.09;
  
	// laser measurement matrix
	H_laser_ << 1.0, 0.0, 0.0, 0.0,
							0.0, 1.0, 0.0, 0.0;
	
	// motion model
	ekf_.F_ = MatrixXd(4, 4);
	ekf_.F_ << 1.0, 0.0, 0.0, 0.0,
						 0.0, 1.0, 0.0, 0.0,
						 0.0, 0.0, 1.0, 0.0,
						 0.0, 0.0, 0.0, 1.0;
		
	ekf_.P_ = MatrixXd(4, 4);
	ekf_.P_ << 1.0, 0.0, 0.0, 0.0,
						 0.0, 1.0, 0.0, 0.0,
						 0.0, 0.0, 1000.0, 0.0,
						 0.0, 0.0, 0.0, 1000.0;
	
	ekf_.Q_ = MatrixXd(4, 4);
	

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() 
{}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) 
{


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
	
  if (!is_initialized_)
	{
    
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 0.0, 0.0, 0.0, 0.0;
				

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
		{
      float rho = measurement_pack.raw_measurements_(0);
			float phi = measurement_pack.raw_measurements_(1);
			float rho_dot = measurement_pack.raw_measurements_(2);
			
			ekf_.x_(0) = rho     * cos(phi);
			ekf_.x_(1) = rho     * sin(phi);
			ekf_.x_(2) = rho_dot * cos(phi);
			ekf_.x_(3) = rho_dot * sin(phi);
						
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
		{
      ekf_.x_(0) = measurement_pack.raw_measurements_(0);
			ekf_.x_(1) = measurement_pack.raw_measurements_(1);
			
    }
		previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  	
	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
	previous_timestamp_ = measurement_pack.timestamp_;
	if (dt > 0.001)
	{
		
		ekf_.Update_F_and_Q(dt);
		ekf_.Predict();			
	}
	
  /*****************************************************************************
   *  Update
   ****************************************************************************/

	
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
	{
    // Radar updates		
		
		VectorXd z(3);
		z << measurement_pack.raw_measurements_(0), measurement_pack.raw_measurements_(1), measurement_pack.raw_measurements_(2);
		
		MatrixXd Hj = tools.CalculateJacobian(ekf_.x_);
		ekf_.R_ = R_radar_;
		ekf_.H_ = Hj;
		ekf_.UpdateEKF(z);
  } 
  else
	{
    // Laser updates	
		
		VectorXd z(2);
		z << measurement_pack.raw_measurements_(0), measurement_pack.raw_measurements_(1);
		ekf_.H_ = H_laser_;
		ekf_.R_ = R_laser_;
		ekf_.Update(z);
  }
  

  // print the output
  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
}

#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,	const vector<VectorXd> &ground_truth) {
  
	VectorXd rmse(4);
	rmse << 0.0, 0.0, 0.0, 0.0;
	
	/* check for incorrect size of input */
	if (estimations.size() != ground_truth.size() || estimations.size() == 0)
	{
		std::cout <<"Incompatible size of estimations" << std::endl;
		return rmse;
	}
	
	for (size_t i = 0; i < estimations.size(); ++i)
	{
		
		VectorXd v = estimations[i] - ground_truth[i];
		v = v.array() * v.array();
		
		rmse += v;
	}
	
	rmse = rmse / estimations.size();
	rmse = rmse.array().sqrt();
	
	
	return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  
	
	/* state variables */
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);
	
	
	/* It is not possible to map X state to radar data (rho, phi, rho dot) */
	
	/* Transformation from cartesian to polar coordinates */
	/* h(x') = (sqrt(px² + py²), arctan(py/px), (px*vx + py*vy)/sqrt(px² + py²)) */
	
	// values pre caching
	
	float v1 = px*px + py*py;
	float v2 = sqrt(v1);
	float v3 = v1*v2;
	
	
	MatrixXd Hj(3, 4);
	
	// Division by zero can happen
	// found it after running tests
	if (fabs(v1) < 0.001)
	{
		std::cout << "CalculateJacobian -Error division by zero" << std::endl;		
		return Hj;
	}
	
	
	
	Hj << px/v2, py/v2, 0, 0,
				-py/v1, px/v1, 0, 0,
				py*(vx*py-vy*px)/v3, px*(vy*px-vx*py)/v3, px/v2, py/v2;
				
	return Hj;
}

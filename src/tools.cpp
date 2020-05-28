#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
	VectorXd rmse = VectorXd::Zero(estimations[0].size());
	if(estimations.size() == 0 || estimations.size() != ground_truth.size())
	{
		std::cout << "Invalid input vector size for RMSE calculation!" << std::endl;
		return rmse;
	}

	
	for(int i = 0;i < estimations.size();i++)
	{
		VectorXd dif = estimations[i]-ground_truth[i];
		dif = dif.array()*dif.array();
		rmse += dif;
	}
	rmse = rmse/estimations.size();
	rmse = rmse.array().sqrt();

	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
	// radar is nonlinear, needs Jacobian
	int radar_n = 3;
	int n = 4;
	MatrixXd Hj(radar_n, n);

	float x = x_state[0];
	float y = x_state[1];
	float vx = x_state[2];
	float vy = x_state[3];

	float t1 = x*x+y*y;
	float t2 = sqrt(t1);
	float t3 = pow(t2, 3);
	//check divided by zero
	if(fabs(t1) < 0.00001)
	{
		t1 = 0.00001;
		//std::cout << "Error: division by zero(Jacobian)." << std::endl;
		//return Hj;
	}

	Hj << x/t2, y/t2, 0, 0, -(y/t1), x/t1, 0, 0, y*(vx*y-vy*x)/t3, x*(vy*x-vx*y)/t3, x/t2, y/t2;

	return Hj;


}

#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

FusionEKF fusion;
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
	const vector<VectorXd> &ground_truth)

{

    //Calculate the RMSE here.


	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;
	// check the validity of the following inputs:
	// the estimation vector size should not be zero
	// the estimation vector size should equal ground truth vector size
	if (estimations.size() != ground_truth.size()|| estimations.size() == 0) 
	{
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	//accumulate squared residuals
	for (unsigned int i = 0; i < estimations.size(); ++i) 
	{
		VectorXd residual = estimations[i] - ground_truth[i];
		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}
	//calculate the mean
	rmse = rmse / estimations.size();
	//calculate the squared root
	rmse = rmse.array().sqrt();
	//return the result
	return rmse;
}

/*
MatrixXd Tools::CalculateJacobian() 
{
		MatrixXd Hj(6, 6);
		//compute the Jacobian matrix
		Hj << 1, 0, 0, fusion.dt, 0, 0,//fusion.dt
			0, 1, 0, 0, fusion.dt, 0,
			0, 0, 1, 0, 0, fusion.dt,
			0, 0, 0, 1, 0, 0,
			0, 0, 0, 0, 1, 0,
			0, 0, 0, 0, 0, 1;
		return Hj;
}
*/
//·Ö±ðÈÆz,y,x×ª
VectorXd Tools::Acceleration_orthogonalization(const float &theta, const float &beta, const float &alpha, VectorXd &x)
{
	MatrixXd orthogonalization(3, 3);//rz1(3, 3), ry(3, 3), rz2(3, 3);
	VectorXd aa(3);//aa << 0, 0, 0;
	/*
//	rz1 << 1,  0,          0,
//		  0,  cos(alpha), -sin(alpha),
//		  0,  sin(alpha), cos(alpha);
	rz1 << cos(alpha), -sin(alpha), 0,
		sin(alpha), cos(alpha), 0,
		0, 0, 1;

	ry << cos(beta), 0,   sin(beta),
		  0,         1,   0,
		-sin(beta),   0,   cos(beta);

	rz2 << cos(theta),  -sin(theta), 0,
		  sin(theta),  cos(theta),  0,
		  0,           0,           1;
	orthogonalization = rz1*ry*rz2;
	*/
	//orthogonalization << cos(alpha)*cos(theta)*cos(beta) - sin(alpha)*sin(theta), -cos(alpha)*cos(beta)*sin(theta)-sin(alpha)*cos(theta), cos(alpha)*sin(beta),
	//	                 cos(beta)*cos(theta)*sin(alpha)+ cos(alpha)*sin(theta), cos(theta)*cos(alpha)-sin(alpha)*sin(theta)*cos(beta), sin(alpha)*sin(beta),
	//	                 -sin(beta)*cos(theta), sin(beta)*sin(theta), cos(beta);

	orthogonalization << cos(theta)*cos(beta), sin(theta), -sin(beta),
		sin(theta), cos(theta)*cos(alpha), sin(alpha),
		sin(beta), -sin(alpha), cos(beta)*cos(alpha);
	//orthogonalization = orthogonalization.inverse();
	aa = orthogonalization*x;
	//aa = a;
	return aa;
	//return orthogonalization;
}


#pragma once
#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"
#include "FusionEKF.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
	/**
	* Constructor.
	*/
	Tools();

	/**
	* Destructor.
	*/
	virtual ~Tools();

	/**
	* A helper method to calculate RMSE.
	*/
	VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

	/**
	* A helper method to calculate Jacobians.
	*/
	MatrixXd Tools::CalculateJacobian();

	VectorXd Tools::Acceleration_orthogonalization(const float &theta, const float &beta, const float &alpha,VectorXd &a);

};

#endif /* TOOLS_H_ */

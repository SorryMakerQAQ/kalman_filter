#pragma once
#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF {
public:
	/**
	* Constructor.
	*/
	FusionEKF();

	/**
	* Destructor.
	*/
	virtual ~FusionEKF();

	/**
	* Run the whole flow of the Kalman Filter from here.
	*/
	void ProcessMeasurement(const MeasurementPackage &measurement_pack);

	/**
	* Kalman Filter update and prediction math lives in here.
	*/
	KalmanFilter ekf_;

	long long previous_timestamp_;

	long long dt;

	Eigen::MatrixXd R_GPS_;
	Eigen::MatrixXd R_IMU_;
	Eigen::MatrixXd H_GPS_;
	Eigen::VectorXd v_pre;
	//Eigen::VectorXd angle_pre;//alpha, beta, theta, 
	float alpha_pre;
	float beta_pre;
	float theta_pre;
	//Eigen::VectorXd x_pre;

private:
	// check whether the tracking toolbox was initialized or not (first measurement)
	bool is_initialized_;

	// previous timestamp
	//long long previous_timestamp_;

	// tool object used to compute Jacobian and RMSE
	//Tools tools;

	//Eigen::MatrixXd Hj_;
};

#endif /* FusionEKF_H_ */

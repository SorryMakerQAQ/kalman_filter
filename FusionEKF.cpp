#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
//#include "json.hpp"	

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

//Constructor.
FusionEKF::FusionEKF() 
{
	is_initialized_ = false;
	previous_timestamp_ = 0;
	// initializing matrices
	R_GPS_ = MatrixXd(3,3);
	R_IMU_ = MatrixXd(6, 6);
	H_GPS_ = MatrixXd(3, 6);
	v_pre = VectorXd(3);

    alpha_pre=0;
	beta_pre =0;
	theta_pre=0;

	v_pre << 0, 0, 0;
	//x_pre << 0, 0, 0, 0, 0, 0;
	//measurement covariance matrix - laser
	R_GPS_ << 0.0225, 0, 0,
		0, 0.0225, 0,
		0, 0, 0.0225;

	//measurement covariance matrix - radar
	R_IMU_ << 0.09, 0, 0, 0, 0, 0,
		0, 0.09, 0, 0, 0, 0,
		0, 0, 0.09, 0, 0, 0,
		0, 0, 0, 0.09, 0, 0,
		0, 0, 0, 0, 0.09, 0,
		0, 0, 0, 0, 0, 0.09;

	// Finish initializing the FusionEKF.
	// Set the process and measurement noises
	//create a 4D state vector, we don't know yet the values of the x state
	ekf_.x_ = VectorXd(6);

	//state covariance matrix P
	ekf_.P_ = MatrixXd(6, 6);
	ekf_.P_ << 1, 0, 0, 0, 0, 0,
		0, 1, 0, 0, 0, 0,
		0, 0, 1, 0, 0, 0,
		0, 0, 0, 1000, 0, 0,
		0, 0, 0, 0, 1000, 0,
		0, 0, 0, 0, 0, 1000;

	//measurement matrix
	ekf_.H_ = MatrixXd(3, 6);
	ekf_.H_ << 1, 0, 0, 0, 0, 0,
		0, 1, 0, 0, 0, 0,
		0, 0, 1, 0, 0, 0;

	//the initial transition matrix F_
	ekf_.F_ = MatrixXd(6, 6);
	ekf_.F_ << 1, 0, 0, 1, 0, 0,
		0, 1, 0, 0, 1, 0,
		0, 0, 1, 0, 0, 1,
		0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 1;
}
//Destructor
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) 
{
	//  Initialization
	if (!is_initialized_)
	{
		/*
		  Initialize the state ekf_.x_ with the first measurement.
		  Create the covariance matrix.
		*/
		// first measurement
		cout << "EKF: " << endl;
		ekf_.x_ = VectorXd(6);
		ekf_.x_ << 1, 1, 1 ,1, 1, 1;
		//v_pre = VectorXd(3);
		//v_pre << 0, 0, 0;
		
		if (measurement_pack.sensor_type_ == MeasurementPackage::IMU) 
		{
			//Initialize IMU state.
			ekf_.x_ << 0, 0, 0, measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], measurement_pack.raw_measurements_[2];
			previous_timestamp_ = measurement_pack.timestamp_;
		}
		else if (measurement_pack.sensor_type_ == MeasurementPackage::GPS) 
		{
			//Initialize GPS state.
			ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], measurement_pack.raw_measurements_[2], 0, 0,0;
			previous_timestamp_ = measurement_pack.timestamp_;
			
		}
		// done initializing, no need to predict or update
		is_initialized_ = true;
		return;
	}
	/* 
	   Prediction:
	   Update the state transition matrix F according to the new elapsed time.
	   Time is measured in seconds.
	   Update the process noise covariance matrix.
	   Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
	*/
	//compute the time elapsed between the current and previous measurements
    dt = (measurement_pack.timestamp_ - previous_timestamp_);
	previous_timestamp_ = measurement_pack.timestamp_;

	float dt_2 = dt * dt;
	float dt_3 = dt_2 * dt;
	float dt_4 = dt_3 * dt;

	//set the acceleration noise components
	float noise_ax = 9;
	float noise_ay = 9;
	float noise_az = 9;

	//Modify the F matrix so that the time is integrated
	ekf_.F_(0, 3) = dt;
	ekf_.F_(1, 4) = dt;
	ekf_.F_(2, 5) = dt;

	//set the process covariance matrix Q
	ekf_.Q_ = MatrixXd(6, 6);
	ekf_.Q_ << dt_4 / 4 * noise_ax, 0, 0, dt_3 / 2 * noise_ax, 0, 0,//noise_ax
		0, dt_4 / 4 * noise_ay, 0, 0, dt_3 / 2 * noise_ay, 0,
		0, 0, dt_4 / 4 * noise_az, 0, 0, dt_3 / 2 * noise_az,
		dt_3 / 2 * noise_ax, 0, 0, dt_2*noise_ax, 0, 0,
		0, dt_3 / 2 * noise_ay, 0, 0, dt_2*noise_ay, 0,
		0, 0, dt_3 / 2 * noise_az, 0, 0, dt_2*noise_az;

	ekf_.Predict();
	if (measurement_pack.sensor_type_ == MeasurementPackage::IMU) 
	{
		//imu updates
		ekf_.UpdateEKF(measurement_pack.raw_measurements_,dt);
	}
	else {
		// gps updates
		ekf_.Update(measurement_pack.raw_measurements_);
	}
	// print the output
	cout << "x_:" << ekf_.x_ << endl;
	cout << "P_:" << ekf_.P_ << endl;
	return;
}

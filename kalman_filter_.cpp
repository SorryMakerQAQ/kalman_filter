//1

#include "kalman_filter.h"
#include "tools.h"
#include <iostream>
#include <math.h>
#include "FusionEKF.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;
Tools tools;
FusionEKF fusionekf;
KalmanFilter kalmanfilter;
VectorXd z_pred_ekf;
Eigen::VectorXd v_;
Eigen::VectorXd a_;
Eigen::VectorXd z_;
Eigen::MatrixXd Hj;

//Eigen::Vector3d v_;

//v_ = VectorXd(3);


KalmanFilter::KalmanFilter() {}

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

void KalmanFilter::Predict() {
	/**
	TODO:
	* predict the state
	*/

	x_ = F_ * x_;

	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
	/**
	TODO:
	* update the state by using Kalman Filter equations
	*/


	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + fusionekf.R_GPS_; //
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z, const long long t)
{

	float wx, wy, wz;//角速度
	float alpha, beta, theta;//欧拉角
	float ax, ay, az;//三轴加速度
	float vx, vy, vz;
	ax = z[0];
	ay = z[1];
	az = z[2];
	alpha = z[3];
	beta = z[4];
	theta = z[5];
	wx = z[6];
	wy = z[7];
	wz = z[8];
	v_ = VectorXd(3);
	a_ = VectorXd(3);
	z_ = VectorXd(6);


	a_ << ax, ay, az;
	a_ = tools.Acceleration_orthogonalization(theta, beta, alpha, a_);
	//cout << "v-pre" << fusionekf.v_pre << endl;
	v_ << fusionekf.v_pre[0] + a_[0] * t, fusionekf.v_pre[1] + a_[1] * t, fusionekf.v_pre[2] + a_[2] * t;
	v_ = tools.Acceleration_orthogonalization(wz, wy, wx, v_);
	//cout << "v-" << v_ << endl;
	z_ << v_[0], v_[1], v_[2], a_[0], a_[1], a_[2];
	fusionekf.v_pre = v_;


	Hj = MatrixXd(6, 6);
	Hj << 1, 0, 0, t, 0, 0,//fusion.dt
		0, 1, 0, 0, t, 0,
		0, 0, 1, 0, 0, t,
		0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 1;

	MatrixXd calculate(6, 6);
	calculate << t, 0, 0, 0.5*t*t, 0, 0,
		0, t, 0, 0, 0.5*t*t, 0,
		0, 0, t, 0, 0, 0.5*t*t,
		1, 0, 0, 0, 0, 0,
		0, 1, 0, 0, 0, 0,
		0, 0, 1, 0, 0, 0;

	z_pred_ekf = fusionekf.x_pre + calculate*z_;

	VectorXd y = z_pred_ekf - Hj*x_;
	MatrixXd Hjt = Hj.transpose();
	MatrixXd S = Hj * P_ * Hjt + fusionekf.R_IMU_;

	MatrixXd Si = S.inverse();

	MatrixXd PHt = P_ * Hjt;

	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	fusionekf.x_pre = x_;
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * Hj) * P_;
}

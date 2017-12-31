#include "kalman_filter.h"
#include <iostream>
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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

	// should it be different for laser and radar?
}

VectorXd KalmanFilter::CalculateZ_pred(const VectorXd &z) {
	if (z.size() == 3) {
		float px = x_(0);
		float py = x_(1);
		float vx = x_(2);
		float vy = x_(3);
		float rho = sqrt(px * px + py * py);
		float phi = atan2(py, px); // float phi = atan2(py, px);
		float rho_dot = (px * vx + py * vy) / rho;
		VectorXd z_pred(3);
		z_pred << rho, phi, rho_dot;
		return z_pred;
	}
	else
	{
		VectorXd z_pred = H_ * x_;
		return z_pred;
	}
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
	/**
	TODO:
	* update the state by using Extended Kalman Filter equations
	*/

	VectorXd z_pred = CalculateZ_pred(z);
	cout << "z_pred: " << z_pred << endl;
	VectorXd y = z - z_pred;
	if (z.size() == 3) {
	y(1) = atan2(sin(y(1)), cos(y(1)));
	}
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}



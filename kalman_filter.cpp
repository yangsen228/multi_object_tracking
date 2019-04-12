#include <iostream>
#include "kalman_filter.h"

using namespace std;
using namespace superg;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::init(Eigen::VectorXd x, Eigen::MatrixXd F, Eigen::MatrixXd P, Eigen::MatrixXd Q, Eigen::MatrixXd H, Eigen::MatrixXd R)
{
	x_ = x;
	F_ = F;
	P_ = P;
	Q_ = Q;
	H_ = H;
	R_ = R; 
}

void KalmanFilter::predict()
{
	cout << "predict start" << endl;
	x_ = F_ * x_;
	P_ = F_ * P_ * F_.transpose() + Q_;
	cout << "predict endl;" << endl;
}

Eigen::MatrixXd KalmanFilter::kalmanGain()
{
	cout << "kalmanGain start" << endl;
	Eigen::MatrixXd PH_t = P_ * H_.transpose();
	Eigen::MatrixXd HPH_t = H_ * PH_t;
	Eigen::MatrixXd K_ = PH_t * (HPH_t + R_).inverse();
	cout << "kalmanGain end" << endl;

	return K_;
}

Eigen::VectorXd KalmanFilter::update(const Eigen::VectorXd& z, const Eigen::MatrixXd& R)
{
	cout << "update start" << endl;
	predict();

	// update
	R_ = R;
	Eigen::MatrixXd K_ = kalmanGain();
	
	x_ = x_ + K_ * (z - H_ * x_);

	long x_size = x_.size();
	Eigen::MatrixXd I_ = Eigen::MatrixXd::Identity(x_size, x_size);
	P_ = (I_ - K_ * H_) * P_;
	cout << "update end" << endl;

	return x_;
}

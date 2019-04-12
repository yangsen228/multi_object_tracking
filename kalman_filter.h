#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>

namespace superg
{
	class KalmanFilter
	{
	public:
		// state vector
		Eigen::VectorXd x_;
		// state transition matrix
		Eigen::MatrixXd F_;
		// state covariance matrix
		Eigen::MatrixXd P_;
		// process noise convariance matrix
		Eigen::MatrixXd Q_;
		// measurement transition matrix
		Eigen::MatrixXd H_;
		// measurement covariance matrix
		Eigen::MatrixXd R_;

		KalmanFilter();
		~KalmanFilter();

		// initialization
		void init(Eigen::VectorXd x, Eigen::MatrixXd F, Eigen::MatrixXd P, Eigen::MatrixXd Q, Eigen::MatrixXd H, Eigen::MatrixXd R);

		// kalman filter (prediction + update)
		Eigen::VectorXd update(const Eigen::VectorXd& z, const Eigen::MatrixXd& R);

	private:
		// prediction
		void predict();

		// kalman gain
		Eigen::MatrixXd kalmanGain(); 
	};
}

#endif
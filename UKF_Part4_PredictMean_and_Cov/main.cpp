#include <iostream>
#include <vector>

#include "Eigen/Dense"
#include "ukf.h"


int main() {

	//Create a UKF instance
	UKF ukf;

    //Predict mean and covariance
    Eigen::VectorXd x_pred = Eigen::VectorXd(5);
    Eigen::MatrixXd P_pred = Eigen::MatrixXd(5, 5);
    ukf.PredictMeanAndCovariance(&x_pred, &P_pred);

	return 0;
}

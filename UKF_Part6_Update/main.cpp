#include <iostream>
#include <vector>

#include "Eigen/Dense"
#include "ukf.h"


int main() {

	//Create a UKF instance
	UKF ukf;

	//Update step
    Eigen::VectorXd x_out = Eigen::VectorXd(5);
    Eigen::MatrixXd P_out = Eigen::MatrixXd(5, 5);
    ukf.UpdateState(&x_out, &P_out);

	return 0;
}

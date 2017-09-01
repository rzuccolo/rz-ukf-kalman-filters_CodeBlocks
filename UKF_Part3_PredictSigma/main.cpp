#include <iostream>
#include <vector>

#include "Eigen/Dense"
#include "ukf.h"


int main() {

	//Create a UKF instance
	UKF ukf;

    //Predict sigma points [2n+1,n], n=5
    Eigen::MatrixXd Xsig_pred = Eigen::MatrixXd(15, 5);
    ukf.SigmaPointPrediction(&Xsig_pred);

	return 0;
}

#include <iostream>
#include <vector>

#include "Eigen/Dense"
#include "ukf.h"


int main() {

	//Create a UKF instance
	UKF ukf;

    //Generate augmented sigma points matrix, [2na+1+, na], to na=5+2 on CTRV model
    Eigen::MatrixXd Xsig_aug = Eigen::MatrixXd(15, 7);
    ukf.AugmentedSigmaPoints(&Xsig_aug);

	return 0;
}

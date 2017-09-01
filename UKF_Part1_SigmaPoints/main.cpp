#include <iostream>
#include <vector>

#include "Eigen/Dense"
#include "ukf.h"

int main() {

	//Create a UKF instance
	UKF ukf;

/*******************************************************************************
* Programming assignment calls
*******************************************************************************/

    //Generate sigma points matrix, [2n+1, n], to n=5 on CTRV model
    Eigen::MatrixXd Xsig = Eigen::MatrixXd(11, 5);
    ukf.GenerateSigmaPoints(&Xsig);

    //print result
    std::cout << "Xsig = " << std::endl << Xsig << std::endl;

	return 0;
}

#include <iostream>
#include <vector>

#include "Eigen/Dense"
#include "ukf.h"


int main() {

	//Create a UKF instance
	UKF ukf;

    //Predict Radar measurements, converts prediction to the measurement space, ro/phi/ro_dot
    Eigen::VectorXd z_out = Eigen::VectorXd(3);
    Eigen::MatrixXd S_out = Eigen::MatrixXd(3, 3);
    ukf.PredictRadarMeasurement(&z_out, &S_out);

	return 0;
}

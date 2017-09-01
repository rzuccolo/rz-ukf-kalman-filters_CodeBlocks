#include <iostream>
#include "tools.h"


Tools::Tools() {}

Tools::~Tools() {}

Eigen::VectorXd Tools::CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                              const std::vector<Eigen::VectorXd> &ground_truth) {

  Eigen::VectorXd rmse(6);
  rmse << 0,0,0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if(estimations.size() != ground_truth.size()
		|| estimations.size() == 0){
	std::cout << "Invalid estimation or ground_truth data" << std::endl;
	return rmse;
  }

  //accumulate squared residuals
  for(unsigned int i=0; i < estimations.size(); ++i){

	Eigen::VectorXd residual = estimations[i] - ground_truth[i];

	//coefficient-wise multiplication
	residual = residual.array()*residual.array();
	rmse += residual;
  }

  //calculate the mean
  rmse = rmse/estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;
}

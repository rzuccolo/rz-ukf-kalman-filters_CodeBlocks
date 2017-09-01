#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <math.h>
#include <iomanip>

#include "Eigen/Dense"

#include "measurement_package.h"
#include "ukf.h"
#include "tools.h"


int main()
{
  /* OPEN MEASUREMENTS FILE */
  // Hardcoded input file with laser and radar measurements
  std::string in_file_name_ = "EKF_DATA/obj_pose-laser-radar-synthetic-input.txt";
  std::ifstream in_file(in_file_name_.c_str()); // c_str() converts string to char*
  // Check for error
  if (!in_file.is_open()) {
      std::cout << "Cannot open input file: " << in_file_name_ << std::endl;
  }

  // Create a Kalman Filter instance
  UKF ukf;

  // Used to compute the RMSE later
  Tools tools;
  std::vector<Eigen::VectorXd> estimations;
  std::vector<Eigen::VectorXd> ground_truth;


  /* READ FILE - line by line */
  std::vector<MeasurementPackage> measurement_pack_list; // List of measurment packages (all lines)
  MeasurementPackage meas_package; // Each line data
  long long timestamp; // timestamp data from measurment package (line)
  float x; // x position data from measurment package (line)
  float y; // y position data from measurment package (line)
  float ro; // Radar range
  float theta; // Radar bearing
  float ro_dot; // Radar radial velocity
  float x_gt; // Ground truth
  float y_gt; // Ground truth
  float vx_gt; // Ground truth
  float vy_gt; // Ground truth
  float yaw_gt; // Ground truth
  float yaw_rate_gt; // Ground truth
  std::string line; // It will hold temporary stream line information

  // Prepare to log results
  std::ofstream file_out;
  file_out.open("Output/output.txt");
  file_out << "i" << "\t" << "sen" << "\t "<<"p_x" << "\t" << "p_y" << "\t" << "px_gt" << "\t" << "py_gt" << "\t"
  << "px_sen" << "\t" << "py_sen" << "\t" << "v" << "\t" << "v_gt" << "\t" << "yaw" << "\t" << "yaw_gt" << "\t"
  << "yaw_rate" << "\t" << "yaw_rate_gt" << "\t" << "NIS" << std::endl;

  // Set i to get only first 4 measurements
  int i = 0; // Starting with zero
  while(std::getline(in_file, line) ){ //&& (i<=100)){  // Let's test 100 lines first

      /* READ LINES */
      std::istringstream iss(line);  // istringstream class is used to split line information into variables
      std::string sensor_type;
      iss >> sensor_type;	// Reads first element from the current line
      if(sensor_type.compare("L") == 0){	// Laser measurement, string.compare method will return 0 if true
          // Read measurements
          meas_package.sensor_type_ = MeasurementPackage::LASER;
          meas_package.raw_measurements_ = Eigen::VectorXd(2);
          iss >> x;
          iss >> y;
          meas_package.raw_measurements_ << x,y;
          iss >> timestamp;
          meas_package.timestamp_ = timestamp;
          measurement_pack_list.push_back(meas_package);
          iss >> x_gt;
          iss >> y_gt;
          iss >> vx_gt;
          iss >> vy_gt;
          iss >> yaw_gt;
          iss >> yaw_rate_gt;
          Eigen::VectorXd gt_values(6);
          gt_values(0) = x_gt;
          gt_values(1) = y_gt;
          gt_values(2) = vx_gt;
          gt_values(3) = vy_gt;
          gt_values(4) = yaw_gt;
          gt_values(5) = yaw_rate_gt;
          ground_truth.push_back(gt_values);
      }
      else if(sensor_type.compare("R") == 0){
          meas_package.sensor_type_ = MeasurementPackage::RADAR;
          meas_package.raw_measurements_ = Eigen::VectorXd(3);
          iss >> ro;
          iss >> theta;
          iss >> ro_dot;
          meas_package.raw_measurements_ << ro,theta, ro_dot;
          iss >> timestamp;
          meas_package.timestamp_ = timestamp;
          measurement_pack_list.push_back(meas_package);
          iss >> x_gt;
          iss >> y_gt;
          iss >> vx_gt;
          iss >> vy_gt;
          iss >> yaw_gt;
          iss >> yaw_rate_gt;
          Eigen::VectorXd gt_values(6);
          gt_values(0) = x_gt;
          gt_values(1) = y_gt;
          gt_values(2) = vx_gt;
          gt_values(3) = vy_gt;
          gt_values(4) = yaw_gt;
          gt_values(5) = yaw_rate_gt;
          ground_truth.push_back(gt_values);
      }


          // Call ProcessMeasurment(meas_package) for Kalman filter
    	  ukf.ProcessMeasurement(meas_package);

    	  // Push the current estimated x,y positon from the Kalman filter's state vector
    	  Eigen::VectorXd estimate(6);

    	  double p_x = ukf.x_(0);
    	  double p_y = ukf.x_(1);
    	  double v  = ukf.x_(2);
    	  double yaw = ukf.x_(3);
    	  double yaw_rate = ukf.x_(4);

    	  double v1 = cos(yaw)*v;
    	  double v2 = sin(yaw)*v;

    	  estimate(0) = p_x;
    	  estimate(1) = p_y;
    	  estimate(2) = v1;
    	  estimate(3) = v2;
    	  estimate(4) = yaw;
    	  estimate(5) = yaw_rate;

    	  estimations.push_back(estimate);

    	  Eigen::VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);


      /*
      std::cout << "i=" << i <<", Sen=" << meas_package.sensor_type_ <<
      ", (PXe,PXm,RMSEpx)= (" << p_x << "," << x_gt << "," << RMSE(0) <<
      ", (PYe,PYm,RMSEpy)= (" << p_y << "," << y_gt << "," << RMSE(1) <<
      ", (VXe,VXm,RMSEvx)= (" << v1 << "," << vx_gt << "," << RMSE(2) <<
      ", (VYe,VYm,RMSEvy)= (" << v2 << "," << vy_gt << "," << RMSE(3) <<
      std::endl;
      */

      // Results on screen
      std::cout << "i=" << i <<", Sen=" << meas_package.sensor_type_ <<
      ", NIS_radar_=" << ukf.NIS_radar_ << ", NIS_laser_=" << ukf.NIS_laser_ << std::endl;

      // Log results out
      double NIS = 0;
      if(meas_package.sensor_type_==1) { // Radar
          x  = ro * cos(theta);
          y  = ro * sin(theta);
          NIS = ukf.NIS_radar_;
      }
      else if(meas_package.sensor_type_==0) { // Lidar
          NIS = ukf.NIS_laser_;
      }

      file_out << i << "\t" << meas_package.sensor_type_ << "\t "<< p_x << "\t" << p_y << "\t" << x_gt << "\t" << y_gt << "\t"
      << x << "\t" << y << "\t" << v << "\t" << sqrt(vx_gt*vx_gt + vy_gt*vy_gt) << "\t" << yaw << "\t" << yaw_gt << "\t"
      << yaw_rate << "\t" << yaw_rate_gt << "\t" << NIS << std::endl;

      i++;
  }

  file_out.close();


  /* CLOSE MEASUREMENTS FILE */
  if(in_file.is_open()){
      in_file.close();
  }

  std::cin.get();
  return 0;

}

























































































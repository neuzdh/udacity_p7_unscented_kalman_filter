#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

  // Declare RMSE and set initial values to 0
  VectorXd rmse = VectorXd(4);
  rmse << 0, 0, 0, 0;

  // Check to make sure estimation and ground truth vectors are of same length
  // If not print error and return rmse with value 0, 0, 0, 0.
  if (estimations.size() != ground_truth.size()){

    std::cout << "Error in CalcualteRMSE: Estimation and ground truth data do not match" << std::endl;
    return rmse;
  } else if (estimations.size() == 0) {
    
    std::cout << "Error in CalcualteRMSE: Estimation and ground truth are empty" << std::endl;
    return rmse;
  }

  for (unsigned int i=0; i < estimations.size(); i++){

    // Difference between estimation and ground truth
    VectorXd diff = estimations[i] - ground_truth[i];

    // Perform element wise squaring of the difference
    diff = diff.array()*diff.array();

    // Add the difference to rmse (summation)
    rmse += diff;
  }

  // Divide summation by number of elements and take element-wise sqrt
  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;
}
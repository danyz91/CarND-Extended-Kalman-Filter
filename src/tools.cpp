// Copyright 2020 d.romano991@gmail.com

#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}


VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
    const vector<VectorXd> &ground_truth) {

  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // Checking the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size()
      || estimations.size() == 0) {
    std::cout << "Invalid estimation or ground_truth data" << std::endl;
    return rmse;
  }

  // accumulate squared residuals
  for (unsigned int i=0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];

    // coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  // calculate the mean
  rmse = rmse/estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3, 4);
  Hj = MatrixXd::Zero(3, 4);

  // Recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // Pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);

  // check division by zero of all denominators in Hj formula
  if (fabs(c1) < std::numeric_limits<float>::epsilon()) {
    c1 = std::numeric_limits<float>::epsilon();
  }
  if (fabs(c2) < std::numeric_limits<float>::epsilon()) {
    c2 = std::numeric_limits<float>::epsilon();
  }
  if (fabs(c3) < std::numeric_limits<float>::epsilon()) {
    c3 = std::numeric_limits<float>::epsilon();
  }

  // compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
      -(py/c1), (px/c1), 0, 0,
      py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj;
}


VectorXd Tools::cartesianToPolar(const VectorXd& cart_state) {
  // Getting cartesian state
  float px = cart_state(0);
  float py = cart_state(1);
  float vx = cart_state(2);
  float vy = cart_state(3);

  // Computing rho, phi
  float rho = sqrt(px*px+py*py);
  float phi = atan2(py, px);

  // Check division by zero and compute rho_dot
  if (rho<std::numeric_limits<float>::epsilon()) {
    rho = std::numeric_limits<float>::epsilon();
  }
  float rho_p = (px*vx+py*vy)/rho;

  // Populating h(x)
  VectorXd hx(3);
  hx << rho, phi, rho_p;

  return hx;
}

float Tools::normalize_radians(float in_rad) {
  float res = in_rad;
  if (res > M_PI) {
    // While input radians num is over pi, subtract 2*pi
    while (res > M_PI) {
      res -= 2*M_PI;
    }
  } else if (res < -M_PI) {
    // While input radians num is under -pi, add 2*pi
    while (res <- M_PI) {
      res += 2*M_PI;
    }
  }

  return res;
}


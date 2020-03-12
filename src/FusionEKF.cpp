#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // Initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  // Measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  // Measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  x_ = VectorXd(4);
  H_ = MatrixXd(2, 4);
  F_ = MatrixXd(4, 4);
  Q_ = MatrixXd(4, 4);
  P_ = MatrixXd(4, 4);

  // Masurement matrix
  H_laser_ << 1, 0, 0, 0,
            0, 1, 0, 0;

  // the initial transition matrix F_
  F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;

  Q_ << MatrixXd::Zero(4,4);


  P_ << MatrixXd::Zero(4,4);

  // Noise given
  noise_ax = 9;
  noise_ay = 9;

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {

    // first measurement
    VectorXd first_measurement = measurement_pack.raw_measurements_;

    cout << "EKF: " << endl;

    // Initialze State Covariance Matrix P
    P_ << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1000, 0,
          0, 0, 0, 1000;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      std::cout<<"EKF init with RADAR measurement"<<std::endl;
      float init_rho = first_measurement[0];
      float init_phi = Tools::normalize_radians(first_measurement[1]);
      float init_rho_dot = first_measurement[2];

      //Polar to cartesian conversion.
      float x_comp = init_rho*cos(init_phi);
      float y_comp = init_rho*sin(init_phi);

      // Initialize state.
      x_ << x_comp, y_comp, 0, 0;

      Hj_ = tools.CalculateJacobian(x_);

      // Initialize EKF with Radar Info
      ekf_.Init(x_, P_, F_, Hj_, R_radar_, Q_);

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      std::cout<<"EKF init with LASER measurement"<<std::endl;
      float init_x = first_measurement[0];
      float init_y = first_measurement[1];

      // Initialize state.
      x_ << init_x, init_y, 0, 0;

      // Initialize EKF with Laser Info
      ekf_.Init(x_, P_, F_, H_laser_, R_laser_, Q_);
    }

    // Initialize Timestamp
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;

    return;
  }


  // ########################
  // #####  EKF Predict  #####
  // ########################

  // Computing the time elapsed between the current and previous measurements
  // Time is converted in seconds.
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  // Updating the state transition matrix F integrating the elapsed time.
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // Setting the process covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
         0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
         dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
         0, dt_3/2*noise_ay, 0, dt_2*noise_ay;



  ekf_.Predict();

  // ########################
  // #####  EKF Update  #####
  // ########################

  /**
   * Algorithm
   * 1. Use the sensor type to perform the update step.
   * 2. Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    // Measurement update
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates
    // Measurement update
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}

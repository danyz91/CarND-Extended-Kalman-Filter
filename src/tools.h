#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include <limits>
#include "Eigen/Dense"


class Tools {
 public:
  /**
   * Constructor.
   */
  Tools();

  /**
   * Destructor.
   */
  virtual ~Tools();

  /**
   * A helper method to calculate RMSE.
   */
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, 
                                const std::vector<Eigen::VectorXd> &ground_truth);

  /**
   * A helper method to calculate Jacobians.
   */
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);


  /**
   * A helper method to convert px, py, vx, vy cartesian vector in polar coords.
   */
  static Eigen::VectorXd cartesianToPolar(const Eigen::VectorXd& cart_state);

  /**
   * A helper method to scale rad in range [-pi, pi].
   */
  static float normalize_radians(float in_rad);
};

#endif  // TOOLS_H_

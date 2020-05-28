#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R1, MatrixXd &R2, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_lidar = R1;
  R_radar = R2;
  Q_ = Q_in;
}

// normalize angle (between -pi to pi)
void KalmanFilter::Normalization(double& phi)
{
  phi = atan2(sin(phi), cos(phi));
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  VectorXd mean = VectorXd::Zero(x_.size());
  x_ = F_*x_ + mean;
  P_ = F_*P_*(F_.transpose())+Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  // Lidar
  VectorXd y = z - H_*x_;
  MatrixXd S_ = H_*P_*(H_.transpose()) + R_lidar;
  //Kalman gain
  MatrixXd K_ = P_*(H_.transpose())*(S_.inverse());
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());

  x_ = x_ + K_*y;
  P_ = (I-K_*H_)*P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z, const MatrixXd& Hj_) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  // Radar
  double rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  double phi = atan2(x_(1), x_(0));
  double rho_dot = (x_(0)*x_(2)+x_(1)*x_(3))/rho;
  VectorXd hx = VectorXd(3);
  hx << rho, phi, rho_dot;
  VectorXd y = z - hx;
  Normalization(y(1));
  MatrixXd S_ = Hj_*P_*(Hj_.transpose()) + R_radar;
  //Kalman gain
  MatrixXd K_ = P_*(Hj_.transpose())*(S_.inverse());
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());

  x_ = x_ + K_*y;
  P_ = (I-K_*Hj_)*P_;


}

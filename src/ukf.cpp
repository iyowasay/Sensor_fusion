#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 2;//10*M_PI/180;

  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
  is_initialized_ = false;

  n_x_ = 5;
  n_aug_ = 7;
  n_lidar = 2;
  n_radar = 3;
  num_sig = 2*n_aug_+1; //number of sigma points
  
  previous_time_ = 0.0;
  NIS_radar = 0.0;
  NIS_lidar = 0.0;
  
  //weights
  lambda_ = 3 - n_aug_;
  weights_ = VectorXd(num_sig);
  //weights_(i) = i ==0 ? lambda_/(lambda_+n_aug_):  0.5/(lambda_+n_aug_);
  weights_(0) = lambda_/(lambda_+n_aug_);
  for(int i = 1;i < num_sig;i++)
  {
    weights_(i)= 0.5/(lambda_+n_aug_);
  }
  

  Xsig_pred_ = MatrixXd(n_x_,num_sig);

  Zsig_pred_radar_ = MatrixXd::Zero(n_radar,num_sig);
  S_radar_ = MatrixXd::Zero(n_radar,n_radar);
  z_radar_ = VectorXd::Zero(n_radar);

  //linear measurement transformatrion for lidar
  S_lidar_ = MatrixXd::Zero(n_lidar,n_lidar);
  H_lidar = MatrixXd::Zero(n_lidar,n_x_);
  H_lidar(0,0) = 1;
  H_lidar(1,1) = 1;

  //process and measurement noise covariance
  Q_ = MatrixXd(2,2);// strong approximation(assuming white noise acceleration, but in reality it's randomly changing)
  Q_ << std_a_*std_a_, 0, 0,std_yawdd_*std_yawdd_;
  R_lidar_ = MatrixXd::Zero(n_lidar,n_lidar); 
  R_lidar_(0,0) = std_laspx_*std_laspx_;
  R_lidar_(1,1) = std_laspy_*std_laspy_;
  R_radar_ = MatrixXd::Zero(n_radar,n_radar);
  R_radar_(0,0) = std_radr_*std_radr_;
  R_radar_(1,1) = std_radphi_*std_radphi_;
  R_radar_(2,2) = std_radrd_*std_radrd_;


}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  
  //lidar 
  if(!is_initialized_)
  {
    std::cout << "initializing by lidar and radar data!"<< std::endl;
    is_initialized_ = true;
    x_ = VectorXd::Zero(n_x_);
    P_ = MatrixXd::Identity(n_x_, n_x_);
    if(meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      double px = meas_package.raw_measurements_(0);
      double py = meas_package.raw_measurements_(1);
      x_(0) = px;
      x_(1) = py;
      P_(0,0) = std_laspx_*std_laspx_;
      P_(1,1) = std_laspy_*std_laspy_;
    }
    else if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      double r = meas_package.raw_measurements_(0);
      double phi = meas_package.raw_measurements_(1);
      double r_dot = meas_package.raw_measurements_(2);
      x_(0) = r*cos(phi);
      x_(1) = 0.0 - r*sin(phi);
      x_(2) = r_dot;
      P_(0,0) = std_radr_*std_radr_;
      P_(1,1) = std_radr_*std_radr_;
      P_(2,2) = std_radrd_*std_radrd_;

    }

    previous_time_ = meas_package.timestamp_;
    return;
 
  }
  
  double dt = (meas_package.timestamp_ - previous_time_)/ 1000000.0;
  previous_time_ = meas_package.timestamp_;
  Prediction(dt);
  
  if(meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)
  {
    std::cout << "Update measurement (LIDAR)" << std::endl;
    UpdateLidar(meas_package);
  }
  else if(meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
  {
    std::cout << "Update measurement (RADAR)" << std::endl;
    UpdateRadar(meas_package);
  
  }
  
  


}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

  // augmented x_
  VectorXd x_aug = VectorXd::Zero(n_aug_);
  x_aug.head(n_x_) = x_;
  MatrixXd P_aug = MatrixXd::Zero(n_aug_,n_aug_);
  P_aug.block(0,0,n_x_, n_x_) = P_;
  P_aug(n_x_,n_x_) = std_a_*std_a_;
  P_aug(n_x_+1,n_x_+1) = std_yawdd_*std_yawdd_;


  // generate sigma points
  MatrixXd sigma = MatrixXd(n_aug_, num_sig);
  sigma.col(0) = x_aug;
  double c1 = sqrt(lambda_ + n_aug_);
  MatrixXd A = P_aug.llt().matrixL();
  for(int i = 0;i < n_aug_; i++)
  {
    sigma.col(i+1) = x_aug + c1*A.col(i);
    sigma.col(n_aug_+i+1) = x_aug - c1*A.col(i);

  }

  //put sigma points into process model(CTRV) 
  VectorXd point(n_x_); 
  VectorXd process(n_x_), noise(n_x_);
  for(int i = 0;i < num_sig;i++)
  {
    point = sigma.col(i);

    noise(0) = 0.5*delta_t*delta_t*cos(point(3))*point(5);
    noise(1) = 0.5*delta_t*delta_t*sin(point(3))*point(5);
    noise(2) = delta_t*point(5);
    noise(3) = 0.5*delta_t*delta_t*point(6);
    noise(4) = delta_t*point(6);
    // avoid divided by zero
    if(fabs(point(4)) < 0.001)
    {
      process(0) = point(2)*cos(point(3))*delta_t;
      process(1) = point(2)*sin(point(3))*delta_t;
      process(2) = 0;
      process(3) = 0;
      process(4) = 0;
    }
    else{
      process(0) = point(2)/point(4)*(sin(point(3)+point(4)*delta_t)-sin(point(3)));
      process(1) = point(2)/point(4)*(cos(point(3))-cos(point(3)+point(4)*delta_t));
      process(2) = 0;
      process(3) = point(4)*delta_t;
      process(4) = 0;
      
    }

    Xsig_pred_.col(i) = point.head(n_x_)+process+noise;

  }

  //use Xsig_pred(new set of sigma points) to predict mean covarience martix
  VectorXd x_sum = VectorXd::Zero(n_x_);
  MatrixXd P_sum = MatrixXd::Zero(n_x_,n_x_);
  for(int i = 0;i < num_sig;i++)
  {
    x_sum += weights_(i)*Xsig_pred_.col(i);
  }
  VectorXd diff_x(n_x_);
  for(int i = 0;i < num_sig;i++)
  {
    diff_x = Xsig_pred_.col(i)-x_sum;
    while(diff_x(3) > M_PI) diff_x(3) -=2.*M_PI;
    while(diff_x(3) < -M_PI) diff_x(3) +=2.*M_PI;

    P_sum += weights_(i)*diff_x*diff_x.transpose();
  }
  
  x_ = x_sum;
  P_ = P_sum;

}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
  VectorXd lidar_data = meas_package.raw_measurements_;
  VectorXd z_pred_ = H_lidar*x_;
  VectorXd y = lidar_data - z_pred_;
  S_lidar_ = H_lidar*P_*H_lidar.transpose() + R_lidar_;
  MatrixXd K_lidar = P_*(H_lidar.transpose())*S_lidar_.inverse();
  

  //NIS_lidar.push_back(y.transpose()*S_lidar_.inverse()*y);
  NIS_lidar = y.transpose()*S_lidar_.inverse()*y;
  MatrixXd I = MatrixXd::Identity(n_x_,n_x_);
  x_ = x_ + K_lidar*y;
  P_ = (I - K_lidar*H_lidar)*P_;

  while(x_(3) > M_PI) x_(3) -=2.*M_PI;
  while(x_(3) < -M_PI) x_(3) +=2.*M_PI;

}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

  VectorXd point2 = VectorXd(n_x_);
  for(int j = 0;j < num_sig;j++)
  {
    point2 = Xsig_pred_.col(j);
    Zsig_pred_radar_(0,j) = sqrt(point2(0)*point2(0)+point2(1)*point2(1));
    Zsig_pred_radar_(1,j) = atan2(point2(1),point2(0));
    Zsig_pred_radar_(2,j) = point2(2)*(point2(0)*cos(point2(3))+point2(1)*sin(point2(3)))/Zsig_pred_radar_(0,j);

    //predict measurement mean and covarience
    z_radar_ += weights_(j)*Zsig_pred_radar_.col(j); 
  }

  //predict measurement mean and covarience
  for(int j = 0;j < num_sig;j++)
  {
    VectorXd diff_z = Zsig_pred_radar_.col(j)-z_radar_;
    while(diff_z(1) > M_PI) diff_z(1) -=2.*M_PI;
    while(diff_z(1) < -M_PI) diff_z(1) +=2.*M_PI;
    S_radar_ += weights_(j)*diff_z*diff_z.transpose();
  }
  S_radar_ = S_radar_ + R_radar_;
  
  VectorXd radar_data = meas_package.raw_measurements_;
  MatrixXd T2 = MatrixXd::Zero(n_x_, n_radar);

  VectorXd diff_z(n_radar);
  diff_z = radar_data-z_radar_;
  while(diff_z(1) > M_PI) diff_z(1) -= 2.*M_PI;
  while(diff_z(1) < -M_PI) diff_z(1) += 2.*M_PI;

  // NIS
  NIS_radar = diff_z.transpose()*S_radar_.inverse()*diff_z;
  //NIS_radar.push_back(diff_z.transpose()*S_radar_.inverse()*diff_z);

  for(int i = 0;i < num_sig;i++)
  {
    VectorXd d1 = Xsig_pred_.col(i) - x_;
    while(d1(3) > M_PI) d1(3) -=2.*M_PI;
  	while(d1(3) < -M_PI) d1(3) +=2.*M_PI;
    
    VectorXd d2 = Zsig_pred_radar_.col(i)-z_radar_;
    while(d2(1) > M_PI) d2(1) -=2.*M_PI;
  	while(d2(1) < -M_PI) d2(1) +=2.*M_PI;

    T2 += weights_(i)*d1*d2.transpose();
  }
  
  MatrixXd K2 = T2*S_radar_.inverse();
  x_ = x_ + K2*diff_z;
  P_ = P_ - K2*S_radar_*K2.transpose();



}
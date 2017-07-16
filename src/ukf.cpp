#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include "tools.h"
#include <list>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {

  //Not initialized yet
  is_initialized_ = false;
  
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  //Set state dimension
  n_x_ = 5;

  // initial state vector
  x_ = VectorXd(n_x_);
  x_.fill(0.0);

  // initial covariance matrix, indentity matrix
  P_ =  MatrixXd::Identity(n_x_, n_x_);
  P_.fill(0.0);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3;//2

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = M_PI/6;//3 //1 //30 degrees

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

  //Set augmented state dimension
  n_aug_ = 7;

  //Sigma spreading parameter
  lambda_ = 3 - n_aug_;

  //Weights of sigma points
  weights_ = VectorXd(2*n_aug_+1);
  //Set weights values
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights_(0) =  weight_0;
  for(unsigned int i=1; i < 2*n_aug_+1; i++){
    double weight = 0.5/(n_aug_ + lambda_);
    weights_(i)= weight;
  }

  //Sigma points
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_ + 1);
  Xsig_pred_.fill(0.0);

  //NIS list values
  nis_radar_ = list<double>();
  nis_laser_ = list<double>();

  /**
   * Choose to print or not the nis values graph
   * Graph are saved in the same folder with the names:
   * 1- nis_radar.png 2- nis_laser.png
   * Requires Gnuplot to print the graph
   */  
  print_graph_ = false;

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {  

  /*********************
   * Check Sensor type *
   *********************/

  if(!use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR){
    //Ignore radar reading
    cout << "Ignore radar reading" << endl;
    return;
  } else if(!use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER){
    //Ignore lidar reading
    cout << "Ignore lidar reading" << endl;
    return;
  }  

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {

    cout << "Initialize Filter\n";
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {      
      /**
       * Convert radar from polar to cartesian coordinates and initialize state.
       * speed can be ignored for the first read 
      */
      double rho    = meas_package.raw_measurements_(0);
      double phi = meas_package.raw_measurements_(1);
      
      //convert to cartesian coordinates
      double px = rho * cos(phi);
      double py = rho * sin(phi);      
      x_ << px, py, 0.0, 0.0, 0.0;
    } else {
      double px = meas_package.raw_measurements_(0);
      double py = meas_package.raw_measurements_(1);
      x_ << px, py, 0.0, 0.0, 0.0;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;

    //store timestamp
    time_us_ = meas_package.timestamp_;

    cout << "Finish first measure\n";
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  cout << "PREDICT\n";
  //time is measured in seconds
  double dt = (meas_package.timestamp_ - time_us_)/ 1000000.0L;
  time_us_ = meas_package.timestamp_;

  //Generate Sigma points and predict state mean and covariance
  Prediction(dt);

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  cout << "UPDATE\n";
  
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    //Radar update
    cout << "RADAR UPDATE\n";
    UpdateRadar(meas_package);
  } else {    
    //Lidar update
    cout << "LIDAR UPDATE\n";
    UpdateLidar(meas_package);
  }
  
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {

  //helper variable
  double delta_t2 = delta_t*delta_t;  
  
  //Augment sigma points
  VectorXd x_aug = VectorXd(n_aug_);
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_+1);

  //create augmented mean state
  x_aug.fill(0.0);
  x_aug.head(n_x_) = x_;

  //create augmented covariance state
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.fill(0.0);
  Xsig_aug.col(0) = x_aug;
  for(unsigned int i=0; i < n_aug_; i++){
    Xsig_aug.col(i+1)        = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }

  //predict sigma points
  for(unsigned int i=0; i < (2*n_aug_+1); i++){
    double p_x  = Xsig_aug(0,i);
    double p_y  = Xsig_aug(1,i);
    double v    = Xsig_aug(2,i);
    double yaw  = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //helper variables
    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);

    //predict state values
    double px_p, py_p;

    //check division by zero
    if(fabs(yawd) > 0.001){
      px_p = p_x + v/yawd * ( sin(yaw + yawd*delta_t) - sin_yaw);
      py_p = p_y + v/yawd * ( cos_yaw - cos(yaw + yawd*delta_t));      
    } else {
      px_p = p_x + v*delta_t*cos_yaw;
      py_p = p_y + v*delta_t*sin_yaw;
    }

    double v_p    = v;
    double yaw_p  = yaw + yawd*delta_t;
    double yawd_p = yawd;

    
    //add noise
    px_p  = px_p + 0.5*nu_a*delta_t2*cos_yaw;
    py_p  = py_p + 0.5*nu_a*delta_t2*sin_yaw;
    v_p   = v_p  + nu_a*delta_t;
    yaw_p  = yaw_p + 0.5*nu_yawdd*delta_t2;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write sigma points back
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
      
  }

  //Predict mean and covariance
  //weights are already set (see constructor)
  //predict state mean
  x_.fill(0.0);
  for(unsigned int i=0; i < 2*n_aug_+1; i++){
    //iterate over sigma points
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  //predict state covariance matrix
  P_.fill(0.0);
  for(unsigned int i=0; i < 2*n_aug_+1; i++){
    //iterate over sigma points
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while(x_diff(3) >  M_PI) x_diff(3)-=2.*M_PI;
    while(x_diff(3) < -M_PI) x_diff(3)+=2.*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
  
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  
  //z vector
  const int n_z = 2;
  VectorXd z = VectorXd(n_z);
  z(0) = meas_package.raw_measurements_(0);
  z(1) = meas_package.raw_measurements_(1);

  /***************
   * MEASUREMENT *
   ***************/

  //sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);

  //transform sigma points into measurement space
  for(unsigned int i=0; i < 2*n_aug_+1; i++){

    //helper variables
    const double p_x = Xsig_pred_(0,i);
    const double p_y = Xsig_pred_(1,i);

    //measurement model
    Zsig(0,i) = p_x; 
    Zsig(1,i) = p_y;
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for(unsigned int i=0; i < 2*n_aug_+1; i++){
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for(unsigned int i=0; i< 2 * n_aug_ + 1; i++){
    //helper variable
    VectorXd diff = Zsig.col(i) - z_pred;
    
    S = S + weights_(i) * diff * diff.transpose();
  }

  //add noise to covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_laspx_*std_laspx_, 0,
    0, std_laspy_*std_laspy_;
    
  S = S + R;

  /**********
   * UPDATE *
   **********/

  //compute corss correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for(unsigned int i=0; i<2*n_aug_+1; i++){

    //helper variable
    VectorXd z_diff = Zsig.col(i) - z_pred;
    
    //state diff
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //compute kalman gain K
  MatrixXd S_inverse = S.inverse();
  MatrixXd K = Tc * S_inverse;

  //helper variable
  VectorXd z_diff = z - z_pred;
  
  //update state mean and covariance
  x_ = x_ + K * z_diff;
  P_  = P_ - K * S * K.transpose();

  //compute nis
  double nis = Tools::CalculeNIS(z_diff, S_inverse);
  nis_laser_.push_back(nis);
  cout << "NIS Lidar: " << nis <<  endl;

  //plot Graph
  PrintNIS(MeasurementPackage::LASER);
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {

  //z vector
  const int n_z = 3;
  VectorXd z = VectorXd(n_z);
  z(0) = meas_package.raw_measurements_(0);
  z(1) = meas_package.raw_measurements_(1);
  z(2) = meas_package.raw_measurements_(2);

  /***************
   * MEASUREMENT *
   ***************/  

  //sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);

  //transform sigma points into measurement space
  for(unsigned int i=0; i < 2*n_aug_+1; i++){

    //helper variables
    const double p_x = Xsig_pred_(0,i);
    const double p_y = Xsig_pred_(1,i);
    const double v   = Xsig_pred_(2,i);
    const double yaw = Xsig_pred_(3,i);

    const double cos_v = cos(yaw)*v;
    const double sin_v = sin(yaw)*v;

    const double sq_ps = sqrt(p_x*p_x + p_y*p_y);

    //measurement model
    Zsig(0,i) = sq_ps;                         //r
    Zsig(1,i) = atan2(p_y, p_x);               //phi
    Zsig(2,i) = (p_x*cos_v + p_y*sin_v)/sq_ps; //r_dt
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for(unsigned int i=0; i < 2*n_aug_+1; i++){
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for(unsigned int i=0; i< 2 * n_aug_ + 1; i++){
    //helper variable
    VectorXd diff = Zsig.col(i) - z_pred;

    //angle normalization
    while(diff(1) > M_PI) diff(1)-=2.*M_PI;
    while(diff(1) <-M_PI) diff(1)+=2.*M_PI;

    S = S + weights_(i) * diff * diff.transpose();
  }

  //add noise to covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_radr_*std_radr_, 0, 0,
    0, std_radphi_*std_radphi_, 0,
    0, 0, std_radrd_*std_radrd_;
  S = S + R;

  /**********
   * UPDATE *
   **********/

  //compute corss correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for(unsigned int i=0; i<2*n_aug_+1; i++){

    //helper variable
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while(z_diff(1) > M_PI) z_diff(1)-=2.*M_PI;
    while(z_diff(1) <-M_PI) z_diff(1)+=2.*M_PI;

    //state diff
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    //angle normalization
    while(x_diff(1) > M_PI) x_diff(1)-=2.*M_PI;
    while(x_diff(1) <-M_PI) x_diff(1)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //compute kalman gain K
  MatrixXd S_inverse = S.inverse();
  MatrixXd K = Tc * S_inverse;

  //helper variable
  VectorXd z_diff = z - z_pred;
  
  //angle normalization
  while(z_diff(1) > M_PI) z_diff(1)-=2.*M_PI;
  while(z_diff(1) <-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean and covariance
  x_ = x_ + K * z_diff;
  P_  = P_ - K * S * K.transpose();

  //compute nis
  double nis = Tools::CalculeNIS(z_diff, S_inverse);
  nis_radar_.push_back(nis);
  cout << "NIS Radar: " << nis <<  endl;

  //plot Nis values
  PrintNIS(MeasurementPackage::RADAR);
}

/**
 * Print Graph with NIS values
 * @param sensor Sensor NIS value to print
 */
void UKF::PrintNIS(MeasurementPackage::SensorType sensor){
  
  //check if graph is enable
  if(!print_graph_){
    cout << "Graphs are not enabled\n";
    return;
  }
  
  //common string values
  const string xTitle = "time";
  const string yTitle = "NIS values";
  
  if (use_radar_ && sensor == MeasurementPackage::RADAR) {
    //print RADAR nis values
    const string title = "RADAR NIS Values";
    const string fileName = "nis_radar.png";
    //do not print graph until more than 250 nis values are available
    if(nis_radar_.size() < 240)
      return;
    Tools::PrintGraph(nis_radar_, fileName, 7.815, title, xTitle, yTitle);
  } else if(use_laser_){
    //print Lidar nis values
    const string title = "LASER NIS Values";
    const string fileName = "nis_laser.png";
    //do not print graph until more than 250 nis values are available
    if(nis_laser_.size() < 240)
      return;
    Tools::PrintGraph(nis_laser_, fileName, 5.991, title, xTitle, yTitle);
  }
}

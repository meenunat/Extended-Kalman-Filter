#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

const float DoublePI = 2 * M_PI;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_; 

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

  VectorXd y = z -  H_ * x_; //z_pred
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  //new state
  x_ = x_ + (K * y);

  P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

   //recover state parameters
   float px = x_(0);
   float py = x_(1);
   float vx = x_(2);
   float vy = x_(3);

   //pre-compute a set of terms to avoid repeated calculation
   float rho = sqrt(px*px+py*py);
   float theta = atan2(py,px);
  // if rho is very small, set it to 0.0001 to avoid division by 0 in computing rho_dot
  if(rho < 0.000001){
	rho = 0.000001;
   }
   float rho_dot = (px*vx+py*vy)/rho;
   VectorXd z_pred = VectorXd(3);
   z_pred << rho, theta, rho_dot;

  VectorXd y = z - z_pred;
  
  // normalize the angle between -pi to pi
  while(y(1) > M_PI){
    y(1) -= DoublePI;
  }
  while(y(1) < -M_PI){
    y(1) += DoublePI;
  }
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  //new state
  x_ = x_ + (K * y);

  P_ = (I - K * H_) * P_; 

}

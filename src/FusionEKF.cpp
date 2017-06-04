#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;




/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;
  
  //create a 4D state vector, we don't know yet the values of the x state
  ekf_.x_ = VectorXd(4);
  
  //the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
		  0, 1, 0, 1,
		  0, 0, 1, 0,
		  0, 0, 0, 1;
		  
  //state covariance matrix P
   ekf_.P_ = MatrixXd(4, 4);
   ekf_.P_ << 1, 0, 0, 0,
		 0, 1, 0, 0,
		 0, 0, 1000, 0,
		 0, 0, 0, 1000;		  

// Laser covariance matrix 
  R_laser_ = MatrixXd(2, 2);
  R_laser_ << 0.0225, 0,
		0, 0.0225;
  
  // Radar covariance matrix
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
  
  // Laser measurement matrix
  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
			  0, 1, 0, 0;
			  
  // Radar measurement matrix
  Hj_ = MatrixXd(3, 4);
  
  
  //set the acceleration noise components
  noise_ax = 8;
  noise_ay = 8;

 
 }

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    //cout << "EKF initialization: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      
      //Convert radar from polar to cartesian coordinates and initialize state.
       // first measurement
      cout << "EKF initialization, Radar: " << endl;
      float px = measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]);
      float py = measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]);
      float vx = measurement_pack.raw_measurements_[2] * cos(measurement_pack.raw_measurements_[1]);
      float vy = measurement_pack.raw_measurements_[2] * sin(measurement_pack.raw_measurements_[1]);
      
      ekf_.x_ << px, py, vx, vy;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
	 // Directly use LIDAR data
      cout << "EKF initialization, LIDAR: " << endl;
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    //cout << ekf_.x_<<endl;
	previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  /* 
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR){
	  cout << "Sensor Type: RADAR" << endl;
	  }
  else{
	  cout << "Sensor Type: LIDAR" << endl;}	  
  */
  
  //compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  //cout << "dt= " << dt << " second(s)" << endl;
  previous_timestamp_ = measurement_pack.timestamp_;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

	//Modify the F matrix so that the time is integrated
   ekf_.F_(0, 2) = dt;
   ekf_.F_(1, 3) = dt;

	//set the process covariance matrix Q, Eq.(40) in the notes
   ekf_.Q_ = MatrixXd(4, 4);
   ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
			   0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
			   dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
			   0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

   //cout << "before Predict(), kf_.x_= " << ekf_.x_ <<endl;
   // Precition after updating F and Q
   ekf_.Predict();
   
   //cout << "after Predict(), kf_.x_= " << ekf_.x_ << endl;
  /*****************************************************************************
   *  Update
   ****************************************************************************/

 if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Calucalte Hj_ based on predicted ekf_.x_
    Tools tools;
	Hj_= tools.CalculateJacobian(ekf_.x_);	  
    //cout << "Radar, Hj_= " << Hj_ << endl;
    // Convert ekf_.x_ to polar coordinates
    VectorXd z_pred;
	z_pred = tools.ConvertToPolar(ekf_.x_);
	
	
	// Proper initiation with Hj_ and R_radar_
	ekf_.Init(ekf_.x_, ekf_.P_, ekf_.F_, Hj_, R_radar_, ekf_.Q_);
	 // Radar measurement update
	ekf_.UpdateWithAlreadyPredictedMeasurements(measurement_pack.raw_measurements_, z_pred);
    } 
  
  else {
    //cout << "Laser, H_laser_= " << H_laser_ << endl;
    // Proper initiation with H_laser_ and R_laser_
	ekf_.Init(ekf_.x_, ekf_.P_, ekf_.F_, H_laser_, R_laser_, ekf_.Q_);
    //Laser measurement update
	ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
}

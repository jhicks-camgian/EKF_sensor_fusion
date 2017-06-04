# Project 1: Extended Kalman Filter 


## C++ IDE and Dependencies

I use mostly Geany IDE, with the following specifications in Build\Set Build Commands as follows:

* Compiler: `g++ -Wall -c "%f" -std=c++11`
* Builder: `g++ -Wall -o "%e"  *.cpp -std=c++11`

The Udacity dependencies require the following:

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4

## Files Submitted
All the submitted files are included in two directory: \src and \build

In the \src directory, the following key files are included:

* main.cpp and main.h \- the main file and the associated header file that implement the reading the input data from files, Kalman Filter (KF) and extended Kalman Filter (EKF), and the calculation of the the root mean square error (RMSE) of the tracking results compared with the ground truth. 
* FusionEKF.cpp and FusionEKF.h \- the file and the associated header file that implement Fusion of measurements from light detection and ranging (LIDAR) and radio detection and ranging (RADAR).
* kalman\_filter.cpp and kalman\_filter.h  \- the file and the associated header file that implement the KF and EKF.
* tool.cpp and tool.h \- the utility file and the associated header file that implement functions used by other files.
* sample-laser-radar-measurement-data-1.txt, and sample-laser-radar-measurement-data-2.txt \- data files that include the measurements and ground truth.
* output1.txt and output2.txt \- the output text files for first and the second measurement files.
* Eigen libraries in the directory \Eigen.
* Executable file main.exe 

In the \build directory, the following key files are included:

* ExtendedKF.exe, the executable file that the main file and the associated header file that implement the reading the input data from files, extended Kalman Filter, and the calculation of the the RMSE of the tracking results compared with the ground truth. 
* CMakeList.txt - build instruction for generating executable.
* sample-laser-radar-measurement-data-1.txt, and sample-laser-radar-measurement-data-2.txt \- data files that include the measurements and ground truth.
* output1.txt and output2.txt \- the output text files for first and the second measurement files.

## Basic Build Instructions Required by Udacity and Alternative Executable File

1. Make a build directory: `mkdir build && cd build`
2. Compile: `cmake .. && make`
3. Run it: `./ExtendedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./ExtendedKF ../data/obj_pose-laser-radar-ekf-input.txt output.txt`

Alternatively, I provide a separate executable main.exe. Run the following to check the results. 
First change to directory \src, then run:

`./main ./sample-laser-radar-measurement-data-1.txt output1.txt` 

and 

`./main ./sample-laser-radar-measurement-data-2.txt  output2.txt`.

## Results
For the data file: sample-laser-radar-measurement-data-1.txt

```
EKF initialization, Radar: 
Accuracy - RMSE:
0.0689231
0.0638141
0.547187
0.553835
```

For the data file: sample-laser-radar-measurement-data-2.txt

```
EKF initialization, LIDAR: 
Accuracy - RMSE:
0.184238
0.191127
0.497462
0.778267
```

##  Notes

For most of the part, I follow strictly the starter code provided by Udactiy. However, I made the following ``educated'' adjustments to meet the requirement mentioned in the project "Tips and Tricks".
 
* In the project "Tips and Tricks", the following is mentioned:  "The R matrix values and Q noise values are provided for you. There is no need to tune these parameters for this project. In the unscented Kalman Filter lectures, we'll discuss how to determine these parameters". However, I used the following noise parameters.

```
//set the acceleration noise components
  noise_ax = 8;
  noise_ay = 8;
```
The results are better than using the default noise parameters.

* In the project "Tips and Tricks", the following is mentioned:
"For lidar measurements, the error equation is y = z - H * x'. For radar measurements, the functions that map the x vector [px, py, vx, vy] to polar coordinates are non-linear. Instead of using H to calculate y = z - H * x', for radar measurements you'll have to use the equations that map from cartesian to polar coordinates: y = z - h(x')". To this end, I implement a simple function in tool.cpp.

```
VectorXd Tools::ConvertToPolar(const VectorXd &x_state) {
	VectorXd  z_pred(3);
	z_pred << 0, 0, 0;
	
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);
    float c = sqrt(px*px+py*py);
    
	// Safeguard against division by zero
	if (fabs(c) < 0.0001){
		cout << "ConvertToPolar() -Error -Division by Zero" << endl;
		c=0.0001;
		}
	
	z_pred << c, atan2(py, px), (px*vx+py*vy)/c;
	
	return z_pred;
}
```
* In the project "Tips and Tricks", the following is mentioned: "In C++, atan2() returns values between -pi and pi. When calculating phi in y = z - h(x) for radar measurements, the resulting angle phi in the y vector should be adjusted so that it is between -pi and pi. The Kalman filter is expecting small angle values between the range -pi and pi". I implement the following in kalman_filter.cpp:

```
if (y[1] < -PI){
	    y[1]=2*PI+y[1];
	}
  else if (y[1] > PI){
        y[1]=2*PI-y[1];
    }
  else
    {
    }
```
* In the project "Tips and Tricks", the following is mentioned: "Before and while calculating the Jacobian matrix Hj, make sure your code avoids dividing by zero. For example, both the x and y values might be zero or px\*px + py\*py might be close to zero. What should be done in those cases?". I implement the following in tools.cpp, following the discussion in: // https://discussions.udacity.com/t/action-on-divide-by-zero-in-jacobian/229082 :

```
//Safeguard against division by zero
if(fabs(c1) < 0.0001){
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
		 Hj <<    0,    0, 0, 0,
                1e+9, 1e+9, 0, 0,
                  0,    0, 0, 0;
       return Hj;          
	}
```
and

```
// Safeguard against division by zero
if (fabs(c) < 0.0001){
		cout << "ConvertToPolar() -Error -Division by Zero" << endl;
		c=0.0001;
}
	
z_pred << c, atan2(py, px), (px*vx+py*vy)/c;
```

* The first two measurements in file  sample-laser-radar-measurement-data-2.txt are all zeros, which will cause NaN errors.  I include the following in the main.cpp to address this issue:

```
// reads first element from the current line
    iss >> sensor_type;
    if (sensor_type.compare("L") == 0) {
      // LASER MEASUREMENT

      // read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = VectorXd(2);
      float x;
      float y;
      iss >> x;
      iss >> y;
      // Safeguard against all-zero measurement
      if (x==0 && y==0){data_all_zero=true;
      }
      else{
         meas_package.raw_measurements_ << x, y;
         iss >> timestamp;
         meas_package.timestamp_ = timestamp;
         measurement_pack_list.push_back(meas_package);
        }   
    } else if (sensor_type.compare("R") == 0) {
      // RADAR MEASUREMENT

      // read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::RADAR;
      meas_package.raw_measurements_ = VectorXd(3);
      float ro;
      float theta;
      float ro_dot;
      iss >> ro;
      iss >> theta;
      iss >> ro_dot;
      // Safeguard against all-zero measurement
      if (ro == 0 && theta==0 && ro_dot==0){data_all_zero=true;
      }	  
      else{
          meas_package.raw_measurements_ << ro, theta, ro_dot;
          iss >> timestamp;
          meas_package.timestamp_ = timestamp;
          measurement_pack_list.push_back(meas_package);
      }    
    }
```
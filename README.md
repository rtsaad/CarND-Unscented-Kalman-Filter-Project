# Unscented Kalman Filter Project

This project consists of a c++ implementation of an Unscented Kalman Filter (UKF) to estimate the state (position, velocity and yaw angle) of a moving object using noisy Lidar and Radar sensors. This project improves the estimations by using the Kalman Filter to fuse the sensor readings. 

The main goals of this project is to develop a c++ kalman filter that successfully estimates the position of a moving object from the Udacity Simulator. Figure 1 depicts an example of the filter estimating (green dots) the object position. The RMSE (Root Mean Square Error) values estimates the accuracy of the UKF.

![alt text][image1]

[//]: # (Image References)

[image1]: images/fusion_data1.png "Fusion Sensor Estimation"
[image2]: images/fusion_data2.png "Fusion Sensor Estimation"
[image3]: images/nis_radar.png "RADAR NIS Values"
[image4]: images/nis_laser.png "LIDAR NIS Values"
[image5]: images/radar.png "Radar Sensor Estimation"
[image6]: images/lidar.png "Lidar Sensor Estimation"

## 1.Access 

The source code for this project is available at [project code](https://github.com/otomata/CarND-Unscented-Kalman-Filter-Project).

## 2.Files

The following files are part of this project: 
* ukf.cpp:   Unscented Kalman Filter class definition;
* tools.cpp:       helper functions to compute rmse errors and nis values for filter consistency check;
* images: 
** fusion_data1.png:  Estimation using Lidar and Radar readings for dataset 1;
** fusion_data2.png:  Estimation using Lidar and Radar readings for dataset 2;
** nis_radar.png:  NIS values for radar estimations;
** nis_laser.png:  NIS values for lidar estimations;
** lidar.png:    Estimation using only Lidar readings;
** radar.png: Estimation using only Radar readings;

### Dependency

This project requires the following packages to work:
* Udacity Simulator [https://github.com/udacity/self-driving-car-sim/releases/](https://github.com/udacity/self-driving-car-sim/releases/);
* cmake 3.5 or above;
* make 4.1 or above;
* gcc/g++: 5.4 or above;
* uWebSocketIO;

### WebSocketIO

This project uses the open source package called WebScokectIO to facilitate the communication between the UKF and the Udacity Simulator. To install all the websocketio libs, execute the script ``install-ubuntu.sh`` from the project repository directory.

## 3.How to use this project

To run this project, you first need to compile the code. After the code is compiled, please, run the Udacity simulator and the UnscentedKF binary created at the build folder.

### Compiling and Running

The main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./UnscentedKF
6. Run the Udacity Simulator (./term2_simulator)

## 4.Results


Figure 2 the UKF estimation using only RADAR readings. The RMSE values of 0.23, 0.30, 0.52 and 0.37 show the UKF accuracy to estimate the moving object position and velocity. 

![alt text][image5]


Figure 3 shows the UKF estimation using only LIDAR readings. As we expected from the LIDAR and RADAR covariance matrix, the accuracy is higher using only the LIDAR sensor. The RMSE values of 0.18, 0.15, 0.54 and 0.25 show the UKF accuracy to estimate the moving object position and velocity. 

![alt text][image6]

Finally, Figures 4 and 5 shows the UKF estimation using RADAR and LIDAR readings. For dataset1, the RMSE values of 0.09, 0.08, 0.30 and 0.19 show the high accuracy the Sensor Fusion promotes to estimate the moving object position, velocity and yaw angle. This [video](https://github.com/otomata/CarND-Unscented-Kalman-Filter-Project/images/fused.ogv) presents the UKF estimating the object position (green dots).

![alt text][image1]

Figures 5 depicts the RMSE values for dataset 2. The Unscented Kalman Filter estimates with high accuracy the position, velocity and yaw angle of the moving object.

![alt text][image2]

## 5. Consistency Check

Figures 6 and 7 depict the NIS values for the radar and lidar sensors, respectively. NIS values are used for consistency check of the filter. From the Figures, we can see that just a few values cross the reference line (~6 for lidar and ~8 for radar) which validates the consistency of our filter. 

![alt text][image3]

![alt text][image4]

## 6 Unscented vs Extended Kalman Filter

Comparing the RMSE values for the UKF with our previous [Extended Kalman Filter](https://github.com/otomata/CarND-Extended-Kalman-Filter-Project), UKF and EKF presents similar accuracy for position estimation but UKF delivers a better estimation for the object velocity. 

## 5. Catch the Run Away Car

The UKF implemented here is used at the "Catch the Run Away Car" project (CRAC). The source code for this project is available at [project code](https://github.com/otomata/CarND-Catch-Run-Away-Car-UKF). 

The CRAC project consists of a hunter car chasing a target car using stationary noisy lidar and radar sensors. Our implementation is able to catch the car in less than 4.5 seconds. This [video](https://github.com/otomata/CarND-Unscented-Kalman-Filter-Project/images/catch_car.ogv) presents car capturing the run away car.





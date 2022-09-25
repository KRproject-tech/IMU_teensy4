# IMU_teensy4
## Overview
Quaternion based IMU with Extended Kalman Filter (EKF) for Teensy 4.0 .

BMX055 is employed as the 9-axis motion sensor to measure the angular velocities and the magnetic fields.

## Hardwares
* MCU board : teensy 4.0
* 9DoF motion sensor : BMX055

## Arduino sketches
* test.ino    : main
* EKF.ino     : Extended Kalman filter code to estimate attitude
* BMX055.ino  : Code to measure the angular velocities and the magnetic fields via BMX055
### Required library for arduino IDE
* Eigen
https://www.arduino.cc/reference/en/libraries/eigen/

## Matlab (R2007b - )
* plot_3D_attitude.m  : Code to plot the attitude estimated by IMU

## Demonstration movie
https://youtu.be/UfdOCR3FNwQ

## Image
![imu](https://user-images.githubusercontent.com/114337358/192146023-927ccd81-bf7d-4c9d-a581-a27021840ddb.png)

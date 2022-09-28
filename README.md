# IMU_teensy4
## Overview
Quaternion based IMU with Extended Kalman Filter (EKF) for Teensy 4.0 .

BMX055 is employed as the 9-axis motion sensor to measure the angular velocities and the magnetic fields.

## Hardwares
* __MCU board__ : teensy 4.0
* __9DoF motion sensor__ : BMX055 (https://akizukidenshi.com/catalog/g/gK-13010/)

## Arduino sketches
* __test.ino__    : main
* __EKF.ino__     : Extended Kalman filter code to estimate attitude
* __BMX055.ino__  : Code to measure the angular velocities and the magnetic fields via BMX055
### Required library for arduino IDE
* __Eigen__
https://www.arduino.cc/reference/en/libraries/eigen/

## Matlab (R2007b - )
* __plot_3D_attitude.m__  : Code to plot the attitude estimated by IMU

## Demonstration movie
https://youtu.be/UfdOCR3FNwQ

## Wiring
![図1](https://user-images.githubusercontent.com/114337358/192680988-31aca1a6-85d3-4055-aff8-a52541c61929.png)

## Image
![imu](https://user-images.githubusercontent.com/114337358/192146023-927ccd81-bf7d-4c9d-a581-a27021840ddb.png)

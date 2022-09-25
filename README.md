# IMU_teensy4
## Overview
Quaternion based IMU with Extended Kalman Filter (EKF) for Teensy 4.0 .

BMX055 is employed as the 9-axis motion sensor to measure the angular velocities and the magnetic fields.

## Arduino sketches
* test.ino    : main
* EKF.ino     : Extended Kalman filter code to estimate attitude
* BMX055.ino  : Code to measure the angular velocities and the magnetic fields via BMX055

## Matlab (R2007b - )
* plot_3D_attitude.m  : Code to plot the attitude estimated by IMU

## Demonstration movie
https://youtu.be/UfdOCR3FNwQ

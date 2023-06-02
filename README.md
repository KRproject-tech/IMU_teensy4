# <p align=center>IMU_teensy4</p>

**Communication**

<a style="text-decoration: none" href="https://twitter.com/hogelungfish" target="_blank">
    <img src="https://img.shields.io/badge/twitter-%40hogelungfish-1da1f2.svg" alt="Twitter">
</a>
<p>

**Language**
<p>
<img src="https://cdn.jsdelivr.net/gh/devicons/devicon/icons/arduino/arduino-original-wordmark.svg"  width="60"/>
<img src="https://cdn.jsdelivr.net/gh/devicons/devicon/icons/matlab/matlab-original.svg" width="60"/>
<p>



## Overview
Quaternion based IMU with Extended Kalman Filter (EKF) for Teensy 4.0 .

BMX055 is employed as the 9-axis motion sensor to measure the angular velocities and the magnetic fields.

## Directory    
<pre>
├─Calibration
│  └─Mag_bias_log
└─memo    
</pre>


## Hardwares
* __MCU board__ : teensy 4.0
* __9DoF motion sensor__ : BMX055 (https://akizukidenshi.com/catalog/g/gK-13010/)

## Initial setting for AE-BMX055
![192917955-fb69abee-906f-49f6-9e24-79df39dd9be5](https://user-images.githubusercontent.com/114337358/192918427-d25a55c3-7cfc-4eec-b05a-6da66e5f165d.png)

## Arduino sketches
* __test.ino__    : main
* __EKF.ino__     : Extended Kalman filter code to estimate attitude
* __BMX055.ino__  : Code to measure the angular velocities and the magnetic fields via BMX055
### Required library for arduino IDE
* __Eigen__
https://www.arduino.cc/reference/en/libraries/eigen/

## Matlab (R2007b - )
* __plot_3D_attitude.m__  : Code to plot the attitude estimated by IMU (Please modify the __COM Port Number__ for your PC)
* __plot_Mag_field.m__ in ./Calibration/Mag_bias_log/ : Calibration for a magnetic sensor from logged data; "Mag_bias_log.txt". 

__Data format of "Mag_bias_log.txt"__: 
[time s] [acc x m/s^2] [acc y m/s^2] [acc z m/s^2] [mag x a.u.] [mag y a.u.] [mag z a.u.]     

## Connections

![image](https://user-images.githubusercontent.com/114337358/203757645-d324abc6-b262-4ad3-8e6c-3ea7b578d1d7.png)



## Demonstration movie
https://youtu.be/UfdOCR3FNwQ

## Wiring
![図1](https://user-images.githubusercontent.com/114337358/192680988-31aca1a6-85d3-4055-aff8-a52541c61929.png)

## Image
![imu](https://user-images.githubusercontent.com/114337358/192146023-927ccd81-bf7d-4c9d-a581-a27021840ddb.png)

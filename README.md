# IMU_teensy4
Quaternion based IMU with EKF for Teensy 4.0 .

BMX055 is employed as the 9-axis motion sensor to measure the angular velocities and the magnetic fields.

[Arduino sketches]
test.ino    : main
EKF.ino     : Extended Kalman filter code to estimate attitude
BMX055.ino  : Code to measure the angular velocities and the magnetic fields via BMX055

[Matlab]

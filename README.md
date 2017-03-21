# adafruit_imu
ROS driver for Adafruit 9-DOF IMU Breakout - L3GD20H + LSM303

This ros package provides a ros node that publishes Imu and MagneticField type messages to "imu" and "mag" topics respectively when the IMU is connected to I2C interface on board. Tested on Raspberry Pi 2 Model B V1.1 aka 2B+

This code is adapted using work of tuuzdu: https://github.com/tuuzdu/crab_project

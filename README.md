# IMU_Kalman-filter_MATLAB
Attitude estimation and animated plot using MATLAB Extended Kalman Filter with MPU9250 (9-Axis IMU)

This is a Kalman filter algorithm for 9-Axis IMU sensors. (Accelerometer, Gyroscope, Magnetometer)

You can see graphically animated IMU sensor with data.



# Demonstration

https://youtu.be/NHTlb84UMCk


# Features
1. Animated plot
2. Timeline
3. Hard-iron bias compensation
4. Angular velocity bias compensation
5. EKF based on quarternion dynamics

# Description
MAIN : Main code

EKF : Main EKF function

sampledata : sampledata I used

mpu9250: If you use mpu9250 and arduino, you can get your sampledata by uploading this file to your arduino. I edited this code from http://arduinolearning.com/code/arduino-mpu-9250-example.php


# How to use
**1. Get data from IMU sensor.** 

You should stay put IMU sensor for the first 2~3 seconds(until Gyro_Compen_k, It's for gyro compensation), then rotate it. 

I recommend you to rotate it several times, different directions. (It's good for magnetometer compensation.)

**2. The data must look like this ↓**


'AccX  AccY  AccZ  GyroX  GyroY  GyroZ  MagX  MagY  MagZ time'         (for k=1)

'AccX  AccY  AccZ  GyroX  GyroY  GyroZ  MagX  MagY  MagZ time'         (for k=2)

.

.

.

(time: IMU running time in ms)

If you use arduino, you can copy from serial monitor, and paste it at sampledata.txt

**3. Edit Initializing variables.**

unit_transform_acc : Converting constant for LSB to N/m^2 (I call the raw data's unit as LSB, It could be wrong. The important thing is that you should convert raw data units to SI units.)

unit_transform_gyro : Converting constant for LSB to rad/s

Gyro_Compen_k : 1\~k'th data will be used for gyro compensation. I recommend 30

Mag_Compen_k : 1\~k'th data will be used for magnetometer compensation. I recommend Nsamples

ref_mag : ref_mag'th data will be reference for psi. I recommend below 30

N_Q : N_Q * unit_matrix for Q matrix

N_R : N_R * unit_matrix for R matrix

N_P : N_P * unit_matrix for P matrix

**4. Run**

If the code is slow, you can comment out the 'Graphical Plot & Dynamic Plot'. you will see only graph.

# Reference
https://github.com/pronenewbits/Arduino_AHRS_System

http://arduinolearning.com/code/arduino-mpu-9250-example.php

MPU-9250 Datasheet & Register map

칼만 필터는 어렵지 않아 with MATLAB Examples - 김성필 (Steady seller about Kalman filter written in Korean)


# Closing
If you have better idea or find error, please let me know.

But, I'm not good at english, so I will be grateful if you explain it easily.

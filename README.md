# IMU_Kalman-filter_MATLAB
Kalman filtering (9-Axis) IMU data with MATLAB

This is Kalman filter algorithm for 9-Axis IMU sensors. (Accelerometer, Gyroscope, Magnetometer)
You can see graphically animated IMU sensor with data.


Testing \n
https://youtu.be/NHTlb84UMCk

Demonstration \n
-Will be added soon.


$Features
1. Animated plot
2. Timeline
3. Hard-iron bias compensation
4. Angular velocity bias compensation

#Description
MAIN : Main code
EKF : Main EKF function
sampledata : sampledata I used

#How to use
1. Get data from IMU sensor. You should stay put IMU sensor for the first 5 seconds(It's for gyro compensation), then rotate it. I recommend you to rotate it several times. (It's good for magnetometer compensation.)
2. The data must be 
'AccX  AccY  AccZ  GyroX  GyroY  GyroZ  MagX  MagY  MagZ time' for k=1
'AccX  AccY  AccZ  GyroX  GyroY  GyroZ  MagX  MagY  MagZ time' for k=2
.
.
.
(time: IMU running time in ms)
If you use arduino, you can copy from serial monitor, and paste it at sampledata.txt
3. Edit Initializing variables.
unit_transform_acc : Constant for LSB to N/m^2
unit_transform_gyro : Constant for LSB to rad/s
Gyro_Compen_k : 1\~k'th data will be used for gyro compensation. I recommend 30
Mag_Compen_k : 1\~k'th data will be used for magnetometer compensation. I recommend Nsamples
ref_mag : ref_mag'th data will be reference for psi. I recommend below 30
N_Q : N_Q * unit_matrix for Q matrix
N_R : N_R * unit_matrix for Q matrix
N_P : N_P * unit_matrix for Q matrix
4. run

If the code is slow, you can annotate out the 'Graphical Plot & Dynamic Plot'. 

# Closing

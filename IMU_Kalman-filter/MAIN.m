clc
clear all
close all

%% DATA EXTRACTING
fileID = fopen('sampledata.txt','r');
DATA = fscanf(fileID,'%f',[10 Inf]); % AccX_raw AccY_raw AccZ_raw GyroX_raw GyroY_raw GyroZ_raw MagX_raw MagY_raw MagZ_raw Time(ms)
N = size(DATA);
Nsamples = N(2)-1; %length of DATA
EulerSaved = zeros(Nsamples, 3);

%% INITIALIZING
g = 9.8;
unit_transform_acc = 16384;
unit_transform_gyro = (pi/(180*131));
unit_transform_mag=0.6;
Gyro_Compen_k = 30;
Mag_Compen_k = 1000;
ref_mag = 30;
DATA_SI = (size(DATA));
N_Q = 1;
N_R = 100;
N_P = 1;

%% LSB to SI Unit
for k = 1:Nsamples
    %Acc LSB -> N/m^2
    DATA_SI(1,k)= (g/unit_transform_acc)*DATA(1,k);
    DATA_SI(2,k)= (g/unit_transform_acc)*DATA(2,k);
    DATA_SI(3,k)= (g/unit_transform_acc)*DATA(3,k);    
    %Gyro LSB -> deg/s -> rad/s
    DATA_SI(4,k)= (unit_transform_gyro)*DATA(4,k);
    DATA_SI(5,k)= (unit_transform_gyro)*DATA(5,k);
    DATA_SI(6,k)= (unit_transform_gyro)*DATA(6,k);   
    %Mag LSB -> uT
    DATA_SI(7,k)= unit_transform_mag*DATA(7,k);
    DATA_SI(8,k)= unit_transform_mag*DATA(8,k);
    DATA_SI(9,k)= unit_transform_mag*DATA(9,k);
    %Time ms -> s
    DATA_SI(10,k)= DATA(10,k)/1000;
end

%% Gyroscope Compensation
for k = 1:Gyro_Compen_k
    Bias_GyroX=mean(DATA_SI(4,k));
    Bias_GyroY=mean(DATA_SI(5,k));
    Bias_GyroZ=mean(DATA_SI(6,k));
end

Bias_Gyro = [Bias_GyroX Bias_GyroY Bias_GyroZ]

for k = 1:Nsamples
    DATA_SI(4,k)=DATA_SI(4,k)-Bias_GyroX;
    DATA_SI(5,k)=DATA_SI(5,k)-Bias_GyroY;
    DATA_SI(6,k)=DATA_SI(6,k)-Bias_GyroZ;
end

%% Magnetometer Compensation

for k = 1:Mag_Compen_k
    Y(k,:) = [DATA_SI(7,k)^2+DATA_SI(8,k)^2+DATA_SI(9,k)^2];
    X(k,:) = [DATA_SI(7,k) DATA_SI(8,k) DATA_SI(9,k) 1];
end

N_X = [size(X)];
Xsamples = N_X(2);
Bias_Mag = 0.5*((X'*X)\eye(Xsamples))*X'*Y

for k = 1:Nsamples
    DATA_SI(7,k) = DATA_SI(7,k) - Bias_Mag(1);
    DATA_SI(8,k) = DATA_SI(8,k) - Bias_Mag(2);
    DATA_SI(9,k) = DATA_SI(9,k) - Bias_Mag(3);
end

%% Set Reference Magnetic vector (NORMALIZATION)
M=sqrt(DATA_SI(7,ref_mag)^2+DATA_SI(8,ref_mag)^2+DATA_SI(9,ref_mag)^2);
B=[DATA_SI(7,ref_mag)/M DATA_SI(8,ref_mag)/M DATA_SI(9,ref_mag)/M];

%% EKF Algorithm
for k = 1:Nsamples-1
  %Assignment
  ax=DATA_SI(1,k);
  ay=DATA_SI(2,k);
  az=DATA_SI(3,k);
  p=DATA_SI(4,k);
  q=DATA_SI(5,k);
  r=DATA_SI(6,k);
  mx=DATA_SI(7,k);
  my=DATA_SI(8,k);
  mz=DATA_SI(9,k);
  dt=(DATA_SI(10,k+1)-DATA_SI(10,k));
  %Normalization
  G=sqrt(ax^2+ay^2+az^2);
  M=sqrt(mx^2+my^2+mz^2);
  ax=ax/G; ay=ay/G; az=az/G; mx=mx/M; my=my/M; mz=mz/M;
  %MAIN EKF FUNCTION
  [q0, q1, q2, q3] = EKF(p, q, r, B, mx, my, mz, ax, ay, az, dt, N_Q, N_R, N_P);
  %Conversion to Euler angle
  phi   =  atan2( 2*(q2*q3 + q0*q1), 1 - 2*(q1^2 + q2^2) );
  theta = -asin(  2*(q1*q3 - q0*q2) );
  psi   =  atan2( 2*(q1*q2 + q0*q3), 1 - 2*(q2^2 + q3^2) );
  EulerSaved(k, :) = [ phi theta psi ];
end 

%% Plot
%Radian to Degree
PhiSaved   = EulerSaved(:, 1) * 180/pi;
ThetaSaved = EulerSaved(:, 2) * 180/pi;
PsiSaved   = EulerSaved(:, 3) * 180/pi;
x = [0 0];
y = [-1000,1000];

figure()
P1=plot(DATA_SI(10,:), PhiSaved, 'r');
hold on
P2=plot(DATA_SI(10,:), ThetaSaved, 'b');
P3=plot(DATA_SI(10,:), PsiSaved, 'g');
refline([0 0])
title('Euler Angle (degree)')
Timeline_1 = line('XData',x,'YData',y);
TimeValue_1= xlabel('');
legend([P1 P2 P3],{'Phi', 'Theta', 'Psi'},'Location','northwest','AutoUpdate','off');
axis([0 DATA_SI(10,Nsamples) -300 300])

figure()
subplot(1,3,1)
plot(DATA_SI(10,:),DATA_SI(1,:),'r',DATA_SI(10,:),DATA_SI(2,:),'g',DATA_SI(10,:),DATA_SI(3,:),'b');
refline([0 0])
title('Acceleration (m/s^2)');
Timeline_2 = line('XData',x,'YData',y);
TimeValue_2= xlabel('');
legend({'AccX', 'AccY', 'AccZ'},'Location','northwest','AutoUpdate','off');
axis([0 DATA_SI(10,Nsamples) -20 20])

subplot(1,3,2)
plot(DATA_SI(10,:),DATA_SI(4,:),'r',DATA_SI(10,:),DATA_SI(5,:),'g',DATA_SI(10,:),DATA_SI(6,:),'b');
refline([0 0])
title('Angular velocity (rad/s)')
Timeline_3 = line('XData',x,'YData',y);
TimeValue_3= xlabel('');
legend({'GyroX', 'GyroY', 'GyroZ'},'Location','northwest','AutoUpdate','off');
axis([0 DATA_SI(10,Nsamples) -3 3])

subplot(1,3,3)
plot(DATA_SI(10,:),DATA_SI(7,:),'r',DATA_SI(10,:),DATA_SI(8,:),'g',DATA_SI(10,:),DATA_SI(9,:),'b');
refline([0 0])
title('Magnetic flux density (uT)')
Timeline_4 = line('XData',x,'YData',y);
TimeValue_4= xlabel('');
legend({'MagX', 'MagY', 'MagZ'},'Location','northwest','AutoUpdate','off');
axis([0 DATA_SI(10,Nsamples) -40 40])

%% Graphical Plot & Dynamic Plot
%set IMU Object's vertex
Cuboid_1=[-2,-3,-0.5];
Cuboid_2=[2,-3,-0.5];
Cuboid_3=[2,3,-0.5];
Cuboid_4=[-2,3,-0.5];
Cuboid_5=[-2,-3,0.5];
Cuboid_6=[2,-3,0.5];
Cuboid_7=[2,3,0.5];
Cuboid_8=[-2,3,0.5];
Cuboid=[Cuboid_1; Cuboid_2; Cuboid_3; Cuboid_4; Cuboid_5; Cuboid_6; Cuboid_7; Cuboid_8];
% ?
fac = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
%Plot Object
figure()
view(3)
Obejct_IMU = patch('Faces', fac, 'Vertices', [0, 0, 0]);
Obejct_IMU.FaceColor = 'g';
axis([-5 5 -5 5 -5 5])
title('IMU state')
Realtime = xlabel(' ');
%Animate
for k=1:Nsamples-1
    %Rotation matrix for vertex
    Rz = [cosd(PsiSaved(k)) -sind(PsiSaved(k)) 0; sind(PsiSaved(k)) cosd(PsiSaved(k)) 0; 0 0 1];
    Ry = [cosd(ThetaSaved(k)) 0 sind(ThetaSaved(k)); 0 1 0; -sind(ThetaSaved(k)) 0 cosd(ThetaSaved(k))];
    Rx = [1 0 0; 0 cosd(PhiSaved(k)) -sind(PhiSaved(k)); 0 sind(PhiSaved(k)) cosd(PhiSaved(k))];
    dt=(DATA_SI(10,k+1)-DATA_SI(10,k));
for j=1:8
    %Rotated vertex
    Result_1(j,:,k) = Rx*Ry*Rz*Cuboid(j,:)';
end
%Display realtime object and time
set(Obejct_IMU, 'Vertices', Result_1(:,:,k))
set(Realtime, 'String', sprintf('time = %.2f [s]', DATA_SI(10, k)))
%Display realtime sensor values
set(TimeValue_1, 'String', sprintf('Phi: %.2f  Theta: %.2f  Psi: %.2f', PhiSaved(k), ThetaSaved(k), PsiSaved(k)));
set(TimeValue_2, 'String', sprintf('X: %.2f  Y: %.2f  Z: %.2f', DATA_SI(1,k), DATA_SI(2,k), DATA_SI(3,k)));
set(TimeValue_3, 'String', sprintf('X: %.2f  Y: %.2f  Z: %.2f', DATA_SI(4,k), DATA_SI(5,k), DATA_SI(6,k)));
set(TimeValue_4, 'String', sprintf('X: %.2f  Y: %.2f  Z: %.2f', DATA_SI(7,k), DATA_SI(8,k), DATA_SI(9,k)));
%Display timeline
set(Timeline_1, 'XData', [DATA_SI(10, k) DATA_SI(10, k)]);
set(Timeline_2, 'XData', [DATA_SI(10, k) DATA_SI(10, k)]);
set(Timeline_3, 'XData', [DATA_SI(10, k) DATA_SI(10, k)]);
set(Timeline_4, 'XData', [DATA_SI(10, k) DATA_SI(10, k)]);
% pause(dt)
drawnow
end
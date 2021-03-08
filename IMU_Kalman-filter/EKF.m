function [q0, q1, q2, q3] = EKF(p, q, r, B, mx, my, mz, ax, ay, az, dt, N_Q, N_R, N_P)
%
%
global Q R
global x P
global firstRun

%Initialize
if isempty(firstRun)
    
  Q = N_Q*eye(4);
     
  R = N_R*eye(6);

  x = [1 0 0 0]';  
  P = N_P*eye(4);
  
  firstRun = 1;  
end

%1.Predict xp and Pp
%Calculate F
F = Fjacob(p, q, r, dt);

xp = F*x;
Pp = F*P*F' + Q;

%2.Correct xp and P
%Calculate H
H = Hjacob(x(1),x(2),x(3),x(4), B);

S= (H*Pp*H' + R);
K = Pp*H'*(S\eye(6));

 z = [ax;
    ay;
    az;
    mx;
    my;
    mz];

x = xp + K*(z - H*xp);
P = Pp - K*H*Pp;

x_sc=(x(1)^2+x(2)^2+x(3)^2+x(4)^2)^0.5;

q0 = x(1);
q1 = x(2);
q2 = x(3);
q3 = x(4);

%------------------------------
function F = Fjacob(p, q, r, dt)
%
%
F = zeros(4);

F(1,1)=1;
F(1,2)= -p*dt/2;
F(1,3)= -q*dt/2;
F(1,4)= -r*dt/2;

F(2,1)= p*dt/2;
F(2,2)=1;
F(2,3)= r*dt/2;
F(2,4)=-q*dt/2;

F(3,1)=q*dt/2;
F(3,2)=-r*dt/2;
F(3,3)=1;
F(3,4)=p*dt/2;

F(4,1)=r*dt/2;
F(4,2)=p*dt/2;
F(4,3)=-q*dt/2;
F(4,4)=1;

%------------------------------
function H = Hjacob(qt0, qt1, qt2, qt3, B)
%
%
g=9.8;
H = zeros(6,4);

H(1,1) = -qt2;
H(1,2) = qt3;
H(1,3) = -qt0;
H(1,4) = qt1;

H(2,1) = qt1;
H(2,2) = qt0;
H(2,3) = qt3;
H(2,4) = qt2;

H(3,1) = qt0;
H(3,2) = -qt1;
H(3,3) = -qt2;
H(3,4) = qt3;

H(4,1) = qt0*B(1)+qt3*B(2)-qt2*B(3);
H(4,2) = qt1*B(1)+qt2*B(2)+qt3*B(3);
H(4,3) = -qt2*B(1)+qt1*B(2)-qt0*B(3);
H(4,4) = -qt3*B(1)+qt0*B(2)+qt1*B(3);

H(5,1) = -qt3*B(1)+qt0*B(2)+qt1*B(3);
H(5,2) = qt2*B(1)-qt1*B(2)+qt0*B(3);
H(5,3) = qt1*B(1)+qt2*B(2)+qt3*B(3);
H(5,4) = -qt0*B(1)-qt3*B(2)+qt2*B(3);

H(6,1) = qt2*B(1)-qt1*B(2)+qt0*B(3);
H(6,2) = qt3*B(1)-qt0*B(2)-qt1*B(3);
H(6,3) = qt0*B(1)+qt3*B(2)-qt2*B(3);
H(6,4) = qt1*B(1)+qt2*B(2)+qt3*B(3);
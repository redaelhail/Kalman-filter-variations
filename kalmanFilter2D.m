% A kalman Filter for tracking the altitude of a rocket


function [pos,vel,Px] = kalmanFilter2D(z)
%
%
persistent A H Q R 
persistent x P
persistent firstRun


if isempty(firstRun)
  firstRun = 1;
  
  dt = 0.02; % sec, for sonar
  
  A = [ 1 0 0 dt 0 0 ;
        0 1 0 0 dt 0 ;
        0 0 1 0 0 dt ;
        0 0 0 1 0 0  ;
        0 0 0 0 1 0  ;
        0 0 0 0 0 1  ;]; % Prediction

  H = [1 0 0 0 0 0;
       0 1 0 0 0 0;
       0 0 1 0 0 0;]; % measurement matrix
  
  Q = [ 1 0 0 0 0 0 ;
        0 3 0 0 0 0 ;
        0 0 2 0 0 0 ;
        0 0 0 1 0 0 ;
        0 0 0 0 1 0 ;
        0 0 0 0 0 1 ;]; % Prediction Noise
  R = 10;

  x = [0 0 0 0 0 0]'; % initial state
  P = 2*eye(6); % initial covariance
end

% Kalman filter algorithm

xp = A*x;  
Pp = A*P*A' + Q;    

K = Pp*H'*inv(H*Pp*H' + R);

x = xp + K*(z - H*xp);
P = Pp - K*H*Pp;   

  
pos = x(1:3);
vel = x(3:6);
Px  = P ;
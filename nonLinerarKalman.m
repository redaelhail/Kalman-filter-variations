function [x,Px] = nonLinerarKalman(x,Px,z)
%
persistent A Q R 
%persistent x P
persistent firstRun


if isempty(firstRun)
  firstRun = 1;
  
  dt = 0.02; % sec, for sonar
  % x x' y y' state
  A = [ 1 dt 0 0 ;
        0 1  0 0 ;
        0 0  1 dt;
        0 0  0 1 ]; % Prediction  

  Q = diag([0.02 .001 0.08 .001]); 
  R = diag([5^2 (pi/30)^2]);

  %x = [0.2 0.1 0.1 0]'; % initial state
  %Px = 0.02*eye(4); % initial covariance
end

% Kalman filter algorithm
x = A*x;  
Px = A*Px*A' + Q;    

% this is the jacobian matrix
r = sqrt(x(1)^2 + x(3)^2);
b = atan2(x(3),x(1));
y = [r;b];
H = [cos(b)   0 sin(b)   0;
    -sin(b)/r 0 cos(b)/r 0];

K = Px*H'/(H*Px*H' + R);
x = x + K*(z - y);
Px = (eye(size(K,1)) - K*H)*Px;

% pos = x([1,3]);
% vel = x([2,4]);
% Px  = Px ;
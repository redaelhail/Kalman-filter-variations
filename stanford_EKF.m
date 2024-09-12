function [x,Px] = stanford_EKF(x,Px,z)
%
persistent A Q R 
%persistent x P
persistent firstRun

points = [
    1, 5;
    1, 10;
    1, 16;
    9, 5;
    9, 10;
    9, 16;
    17, 5;
    17, 10;
    17, 16
];


if isempty(firstRun)
  firstRun = 1;
  
  dt = 0.1; % sec, for sonar
  % x x' y y' state
  A = [ 1  dt  0   0 ;
        0 .85  0  .5 ;
        0  0   1  dt;
        0 -0.1 0  .85]; % Prediction  

  Q = diag([0.02 .001 0.08 .001]); 
  R = 0.002 * eye(9);

  %x = [0.2 0.1 0.1 0]'; % initial state
  %Px = 0.02*eye(4); % initial covariance
end

% Kalman filter algorithm
x = A*x;  
Px = A*Px*A' + Q;    

% this is the jacobian matrix
% Calculate the distance from point P to each of the 9 points
y = sqrt(sum((points - [x(1) x(3)]).^2, 2));
% Jacobian
H1 = (x(1) - points(:, 1))./y;
H2 = (x(3) - points(:, 2))./y;
H = [H1 zeros([9,1]) H2 zeros([9,1])];

K = Px*H'/(H*Px*H' + R);
x = x + K*(z - y);
Px = (eye(size(K,1)) - K*H)*Px;
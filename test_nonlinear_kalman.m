% altitude
close all;
clear;
clc;

t = 0.02;
numPoints = 500;
measurement_x = linspace(1, 10, numPoints);
measurement_theta = linspace(-pi,pi/2, numPoints);

noiseMean = 0;       % Mean of the Gaussian noise
noiseStdDev = 0.34;   % Standard deviation of the Gaussian noise
noise_X = noiseMean + noiseStdDev * randn(1, numPoints);
noise_theta = noiseMean + pi/30 * randn(1, numPoints);

measurement_x = measurement_x+noise_X;
measurement_theta = measurement_theta+noise_theta;

position = zeros(2,numPoints);
kalman_error = zeros(2,numPoints);
x=[ 0.5; 0.007; 0.6; 0.007];
Px = diag([0.01 0 0.01 0]);
for i=1:numPoints
    z = [measurement_x(:,i); measurement_theta(:,i)];
    [x,Px] = nonLinerarKalman(x,Px,z);
    position(:,i) = x([1,3]);
    %kalman_error(:,i) = Px(1,1);
end

% Create a figure
figure;
[rx,ry] = pol2cart(measurement_theta, measurement_x);
subplot(2,1,1); % 2 rows, 1 column, 1st subplot
plot(rx, 'r-', 'DisplayName', 'Measurement x');
hold on;
plot(position(1,:), 'b--', 'DisplayName', 'Kalman Filter');
legend show;
xlabel('Index');
ylabel('Position X');
title('X Axis Measurement vs Kalman Filter');

% Plot for the y-axis
subplot(2,1,2); % 3 rows, 1 column, 2nd subplot
plot(ry, 'g--', 'DisplayName', 'Measurement y');
hold on;
plot(position(2,:), 'b--', 'DisplayName', 'Kalman Filter');
legend show;
xlabel('Index');
ylabel('Position Y');
title('Y Axis Measurement vs Kalman Filter');

% Optional: Adjust the layout
sgtitle('Measurement vs Kalman Filter for X, Y, and Z Axes'); % Super title for the entire figure


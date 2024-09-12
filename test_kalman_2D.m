% altitude
close all;
clear;
clc;

t = 0.2;
numPoints = 100;
measurement_x = linspace(1, 10, numPoints);
measurement_y = linspace(10, 60, numPoints);
measurement_z = linspace(10, 15, numPoints);

noiseMean = 0;       % Mean of the Gaussian noise
noiseStdDev = 0.34;   % Standard deviation of the Gaussian noise
noise_X = noiseMean + noiseStdDev * randn(1, numPoints);
noise_Y = noiseMean + 2 * randn(1, numPoints);
noise_Z = noiseMean + noiseStdDev * randn(1, numPoints);

measurement_x = measurement_x+noise_X;
measurement_y = measurement_y+noise_Y;
measurement_z = measurement_z+noise_Z;

position = zeros(3,numPoints);
kalman_error = zeros(3,numPoints);


for i=1:numPoints
    z = [measurement_x(:,i) measurement_y(:,i) measurement_z(:,i)]';
    [pos,vel,Px] = kalmanFilter2D(z);
    position(:,i) = pos;
    %kalman_error(:,i) = Px(1,1);
end

% Create a figure
figure;

% Plot for the x-axis
subplot(3,1,1); % 3 rows, 1 column, 1st subplot
plot(measurement_x, 'r--', 'DisplayName', 'Measurement x');
hold on;
plot(position(1,:), 'b-', 'DisplayName', 'Kalman Filter');
legend show;
xlabel('Index');
ylabel('Position X');
title('X Axis Measurement vs Kalman Filter');

% Plot for the y-axis
subplot(3,1,2); % 3 rows, 1 column, 2nd subplot
plot(measurement_y, 'g--', 'DisplayName', 'Measurement y');
hold on;
plot(position(2,:), 'b-', 'DisplayName', 'Kalman Filter');
legend show;
xlabel('Index');
ylabel('Position Y');
title('Y Axis Measurement vs Kalman Filter');

% Plot for the z-axis
subplot(3,1,3); % 3 rows, 1 column, 3rd subplot
plot(measurement_z, 'm--', 'DisplayName', 'Measurement z');
hold on;
plot(position(3,:), 'b-', 'DisplayName', 'Kalman Filter');
legend show;
xlabel('Index');
ylabel('Altitude');
title('Z Axis Measurement vs Kalman Filter');

% Optional: Adjust the layout
sgtitle('Measurement vs Kalman Filter for X, Y, and Z Axes'); % Super title for the entire figure


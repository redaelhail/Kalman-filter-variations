% altitude
close all;
clear;
clc;

t = 0.2;
numPoints = 100;
measurement = linspace(1, 10, numPoints);
noiseMean = 0;       % Mean of the Gaussian noise
noiseStdDev = 0.7;   % Standard deviation of the Gaussian noise
noise = noiseMean + noiseStdDev * randn(1, numPoints);

measurement = measurement+noise;
position = zeros(1,numPoints);
kalman_error = zeros(1,numPoints);


for i=1:numPoints
    meas = measurement(:,i); 
    [pos,vel,Px] = kalmanFilter1D(meas);
    position(:,i) = pos;
    kalman_error(:,i) = Px(1,1);
end

figure;
plot(measurement, 'r--', 'DisplayName', 'Measurement');
hold on;
plot(position, 'b-', 'DisplayName', 'Kalman Filter');
legend show;
xlabel('Index');
ylabel('Altitude');
figure;
plot(kalman_error, 'g-', 'DisplayName', 'Kalman Error');


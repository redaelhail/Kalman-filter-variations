% altitude
close all;
clear;
clc;

% Define the number of points in the path
num_points = 100;

% Define the center of the circle
center = [10, 10];

% Define the mean radius and its variation
mean_radius = 8;
radius_variation = 1; % Maximum deviation from the mean radius

% Define the range of angles to create an unclosed path
angle_start = 0;          % Starting angle (in radians)
angle_end = pi;           % Ending angle (in radians)

% Generate angles between angle_start and angle_end
theta = linspace(angle_start, angle_end, num_points);

% Generate a varying radius for each angle
radius = mean_radius + radius_variation * randn(1, num_points);

% Calculate the ideal x and y values for the circle with varying radius
x_ideal = center(1) + radius .* cos(theta);
y_ideal = center(2) + radius .* sin(theta);

% Add some random noise to the circle to make it less perfect
noise_level = 0.2; % Controls the amount of noise

x_noisy = x_ideal + noise_level * randn(1, num_points);
y_noisy = y_ideal + noise_level * randn(1, num_points);

% Define the specific points to scatter
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

% Extract x and y coordinates of the scatter points
x_points = points(:, 1);
y_points = points(:, 2);

%
%Initializiation
x = [10 ;0 ;16; 0];
Px = 0.1*eye(4);
kf_state = zeros(2,num_points);

%
for i=1:num_points
    z = sqrt(sum(([x_noisy(i)  y_noisy(i)] - [x_points y_points]).^2, 2));
    [x,Px] = stanford_EKF(x,Px,z);
    kf_state(:,i) = x([1,3]);
end

%
% Plot the noisy circular path with varying radius
figure; % Create a new figure
plot(x_noisy, y_noisy, '-o', 'DisplayName', 'Noisy Circular Path');
hold on; % Hold the current plot
plot(kf_state(1,:), kf_state(2,:), 'r-', 'DisplayName', 'Noisy Circular Path');
hold on;
% Scatter the specific points as stars
scatter(x_points, y_points,100,'r', 'filled', 'DisplayName', 'Specific Points');

% Add labels, legend, and other plot settings
xlabel('X values');
ylabel('Y values');
title('Noisy Unclosed Circular Path with Specific Points');
legend('show');
grid on;
axis equal; % Keep the aspect ratio equal
xlim([0, 20]);
ylim([0, 20]);

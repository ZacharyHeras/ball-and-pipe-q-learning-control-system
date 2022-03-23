% A MATLAB script to control Rowans Systems & Control Floating Ball 
% Apparatus designed by Mario Leone, Karl Dyer and Michelle Frolio. 
% The current control system is a PID controller.
%
% Created by Kyle Naddeo, Mon Jan 3 11:19:49 EST 
% Modified by Ryan Oliver 2/9/2022
% Modified by Zachary heras 2/23/2022
% 
%% Start fresh
close all; clc; clear device; clear;

%% Connect to device
device = serialport('COM10', 19200);

%% Parameters
target      = 0.5;   % Desired height of the ball [m]
sample_rate = 0.01;  % Amount of time between controll actions [s]

%% Give an initial burst to lift ball and keep in air
set_pwm(device, 0); % Bring ball to ground
pause(2); % Pause for 5 seconds

%% Initialize variables  
error       = 0;
error_sum   = 0;
previous_y  = 0;
velocities = (0);
ys         = (0);
time = 1;

%% Blast the ball

action = 500;
set_pwm(device, action); % Implement action

%% Feedback loop
while true
    
    %% Read current height
    [distance, pwm, target, deadpan] = read_data(device);
    y = ir2y(distance); % Convert from IR reading to distance from bottom [m]
    %disp(['y: ', num2str(y)])
    
    %% Calculate current velocity
    velocity = calculate_velocity(previous_y, y, sample_rate);
    velocities(time) = velocity;
    ys(time) = y;
    %disp(['velocity: ', num2str(velocity)]);
    
    %% Set previous_y
    previous_y = y;
    
    %% Calculate errors for PID controller
    error_prev = error;             % D
    error      = target - y;        % P
    error_sum  = error + error_sum; % I
    
    %% Control
    prev_action = action;

        
    % Wait for next sample
    pause(sample_rate)
    
    %% End for loop
    time = time + 1;
    if time*sample_rate > 2
        break;
    end
    set_pwm(device, 2000);
end

n = 1:length(velocities);

subplot(2,1,1)
plot(n, velocities);
hold off
subplot(2,1,2)
plot(n, ys)
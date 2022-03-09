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
sample_rate = 0.25;  % Amount of time between controll actions [s]

%% Give an initial burst to lift ball and keep in air
set_pwm(device, 200); % Bring ball to ground
pause(5); % Pause for 5 seconds

%% Initialize variables
action      = 0; % Same value of last set_pwm   
error       = 0;
error_sum   = 0;
previous_y  = 0;
velocities = [];

%% Flush the device
flush(device)

%% Feedback loop
while true
    %% Read current height
    [distance, pwm, target, deadpan] = read_data(device);
    y = ir2y(distance); % Convert from IR reading to distance from bottom [m]
    disp(['y: ', num2str(y)])
    
    %% Calculate current velocity
    velocity = calculate_velocity(previous_y, y, sample_rate);
    velocities(end+1) = velocity;
    disp(['velocity: ', num2str(velocity)]);
    
    %% Set previous_y
    previous_y = y;
    
    %% Calculate errors for PID controller
    error_prev = error;             % D
    error      = target - y;        % P
    error_sum  = error + error_sum; % I
    
    %% Control
    prev_action = action;
    action = 4000;
    set_pwm(device, action); % Implement action
        
    % Wait for next sample
    pause(sample_rate)
    
    %% End for loop
    if y > 0.9
        break;
    end
        
end

%% Plot velocities to find max velocity
n = 1:1:len(velocities);

plot(len(velocities), velocities)
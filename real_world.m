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
y_goal      = 0.5;   % Desired height of the ball [m]
sample_rate = 0.15;  % Amount of time between controll actions [s]

%% Bring ball to bottom
action = 0;
set_pwm(action);

%% Feedback loop
while true
    
    %% Read current height
    [distance, pwm, target, deadpan] = read_data(device);
    % set previous y
    previous_y = y;
    [y, ~] = ir2y(distance); % Convert from IR reading to distance from bottom [m]
    disp(['y: ', num2str(y)])

    
    if time == 5
        action = 4095;
        set_pwm(device, action)
    end
    
    %% Calculate current velocity
    velocity = calculate_velocity(previous_y, y, sample_rate);
    
    %% Calculate error
    y_difference = ;
    
    %% Control
    prev_action = action;

        
    % Wait for next sample
    pause(sample_rate)
end
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
% q table for desired height
y_goal_q_table = load('q_tables\q_table_0cm.mat', 'q_table');

% amount of time between controller actions
sample_rate = 0.15;

%% Bring ball to bottom for start
action = 0;
set_pwm(action);

%% Ball and pipe control loop
while true
    % set previous y
    previous_y = y;
    
    % read current height
    [distance, pwm, target, deadpan] = read_data(device);
    
    % convert from IR reading to distance from bottom
    [y, ~] = ir2y(distance);

    % calculate current velocity
    velocity = calculate_velocity(previous_y, y, sample_rate);
    
    % calculate discrete state
    discrete_state = ...
        get_discrete_state([y, velocity], [0, -1.5], [0.435, 0.1429]);
    
    % use q table to determine action
    [~, action] = ...
                    max(q_table(discrete_state(1), discrete_state(2), :));
        
    % wait for next sample
    pause(sample_rate)
end
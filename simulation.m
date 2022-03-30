%% Created by Zachary Heras 2/23/2022
clear;

%% Given variables in SI units
% mass of ping pong ball
mass = 2.7 * 10^-3;

% volume of ping pong ball
volume = 3.35 * 10^-5;

% density of air
density = 1.225;

% acceleration of gravity
g = 9.8;

% max height of pipe
max_height = 0.9144;

% wind velocity when ball is in equilibrium
% this is an approximate value
% real value changes with height
velocity_eq = 2.4384;

% sample_rate of controller
sample_rate = 0.25;

% time that simulation will run for
simulation_time = 30;

% number of simulation steps
steps = simulation_time / sample_rate;

%% G1: Wind speed to ball position
syms s;
c2 = ((2*g) / velocity_eq) * (mass - density * 0.00026) / (mass);
Ys = c2; % Just reassigned for nomenclature
Vs = sym2poly(s*(s+c2));
G1 = tf(Ys,Vs);

%% G2: PWM to wind speed
G2 = 6.3787 * 10^-4;

%% G3: PWM to ball position
G3 = G2 * G1;

%% State space representation
G = ss(G3);

%% Checking simulation for accuracy
% initialize state to 0
previous_state = [0; 0];

% initialize y values to 0
y_values = zeros(steps, 1);

% action values for testing
actions = (3000 - 2727.0447) .* ones(steps, 1);
actions(10:end) = (0 - 2727.0447);

%% Simulation
for i = 1 : 1 : (length(y_values))
   
    % new simulation step
    [y_new, previous_time_samples, previous_states] = ...
        lsim(G, [actions(i), actions(i)], [0, sample_rate], previous_state);
    
    % display simulation step values
%     disp('y_new: ');
%     disp(y_new);
%     
%     disp('new_state: ');
%     disp([previous_states(end - 2), previous_states(end)])
    
    previous_state = [previous_states(end - 2), previous_states(end)];
    
    % add new simulation step to previous states
    y_values(i) = y_new(end - 1);
    y_values(i + 1) = y_new(end);
    
    % enforce max height
    if y_new(end - 1) > max_height
        y_values(i) = max_height;
    end
    
    if y_new(end) > max_height
        y_values(i + 1) = max_height;
    end
    
    % enforce min height
    if y_new(end - 1) < 0
        y_values(i) = 0;
    end
    
    if y_new(end) < 0
        y_values(i + 1) = 0;
    end
end

plot(0:sample_rate:simulation_time, y_values);
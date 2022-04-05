% A MATLAB script to test a q table for the ball and pipe system
% Created by Zachary Heras 4/4/2022
%% Clear workspace nad command window
clear, clc;

%% Simulation parameters
% mass of ping pong ball
mass = 2.7 * 10^-3;

% volume of ping pong ball
volume = 3.35 * 10^-5;

% density of air
density = 1.225;

% acceleration of gravity
g = 9.8;

% wind velocity when ball is in equilibrium
% this is an approximate value
% real value changes with height
velocity_eq = 2.4384;

% sampling_rate of controller
sampling_rate = 0.15;

% min height of pipe
min_height = 0;

% max height of pipe
max_height = 0.9144;

% max velocity of ball and pipe system
max_vel = 1.5;

% min velocity of ball and pipe system
min_vel = -1.5;

% max pwm of ball and pipe system
max_pwm = 4095 - 2727.0477;

% min pwm of ball and pip system
min_pwm = 0 - 2727.0477;

% bucket size
bucket_size = 20;

% height space / range
height_space = min_height:...
    ((max_height - min_height) / bucket_size):max_height;
 
% velocity space / range
velocity_space = min_vel:((max_vel - min_vel) / bucket_size):max_vel;

% pwm space / range
pwm_space = min_pwm:((max_pwm - min_pwm) / bucket_size):max_pwm;

% observation space high
os_high = horzcat(max_height, max_vel);

% observation space low
os_low = horzcat(min_height, min_vel);

% observation space window size
os_win_size = (os_high - os_low) ./...
 horzcat(length(height_space), length(velocity_space));

% number of episodes
episodes = 7000;

% time that simulation will run for in seconds
episode_length = 144;

% number of simulation steps
steps = episode_length / sampling_rate;

%% Transfer function
% G1: Wind speed to ball position
syms s;
c2 = ((2*g) / velocity_eq) * (mass - density * 0.00026) / (mass);
Ys = c2; % Just reassigned for nomenclature
Vs = sym2poly(s*(s+c2));
G1 = tf(Ys,Vs);

% G2: PWM to wind speed
G2 = 6.3787 * 10^-4;

% G3: PWM to ball position
G3 = G2 * G1;

% State space representation
G = ss(G3);

%% Simulation test
% hold figure so each episode can be plotted together
% hold on

% q table for desired height
load('q_tables\q_table_max_velocity.mat', 'q_table');

% initialize first discrete state as 0 height and 0 velocity
discrete_state = get_discrete_state([0, 0], os_low, os_win_size);

% initialize ss state
previous_state = [0; 0];

% initialize y values to 0s
y_values = zeros(steps, 1);

% initilize y_previous to 0
y_previous = 0;

% initialize y_current to 0
y_current = [0; 0];

% loop through each time step in each episode
for time = 1 : 1 : (length(y_values))

    % choose action with highest reward
    [current_q, action] = ...
        max(q_table(discrete_state(1), discrete_state(2), :));
    color = '-m';

    % get new simulation step
    [y_current, previous_time_samples, previous_states] = ...
        lsim(G, [pwm_space(action), pwm_space(action)],...
        [0, sampling_rate], previous_state);

    % enforce max height
    if y_current(end - 1) > max_height
        y_current(end - 1) = max_height;
        y_values(time) = max_height;

    end

    if y_current(end) > max_height
        y_current(end) = max_height;
        y_values(time + 1) = max_height;
    end

    % enforce min height
    if y_current(end - 1) < min_height
        y_current(end - 1) = min_height;
        y_values(time) = min_height;
    end

    if y_current(end) < min_height
        y_current(end) = min_height;
        y_values(time + 1) = min_height;
    end

    % calculate velocity
    if y_current == 0 | y_current == max_height %#ok<OR2>
        velocity_current = 0;
    else
        velocity_current = max(calculate_velocity(y_previous,...
        y_current(end), sampling_rate));
    end

    % calculate new discrete step
    new_discrete_state = get_discrete_state([y_current(end),...
        velocity_current], os_low, os_win_size);

    % update previous y value
    y_previous = y_current(end);

    % update discrete_state
    discrete_state = new_discrete_state;

    % extract previous state from previous state vector
    previous_state = [previous_states(end - 2), previous_states(end)];

    % enforce max height on ss model
    % 12.6356 is max height on space state model

    if previous_state(end) > 12.6356
        previous_state(end) = 12.6356;
    end

    % enforce min height on ss model
    % 0 is min height on space state model
    if previous_state(end) < 0
        previous_state(end) = 0;
    end

    % add new simulation step to previous states
    y_values(time) = y_current(end - 1);
    y_values(time + 1) = y_current(end);

end

% visualise q table test simulation
plot(0:sampling_rate:episode_length, y_values, color);
ylim([0 1])
xlabel('Time(s)')
ylabel('Height(m)')
    title('Height vs. Time')
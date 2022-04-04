%% Created by Zachary Heras 2/23/2022
clear, clc;

%% Given variables in SI units and hyperparameters
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
sampling_rate = 0.4;

% time that simulation will run for in seconds
episode_length = 30;

% number of simulation steps
steps = episode_length / sampling_rate;

% bucket length
bucket_size = 20;

% max height of pipe
max_height = 0.9144;

% min height of pipe
min_height = 0;

% max velocity of ball and pipe system
max_vel = 1.5;

% min velocity of ball and pipe system
min_vel = -1.5;

% max pwm of ball and pipe system
max_pwm = 4095 - 2727.0477;

% min pwm of ball and pip system
min_pwm = 0 - 2727.0477;

% height space / range
height_space = min_height:...
    ((max_height - min_height) / bucket_size):max_height;
 
% velocity space / range
velocity_space = min_vel:((max_vel - min_vel) / bucket_size):max_vel;

% pwm space / range
pwm_space = min_pwm:((max_pwm - min_pwm) / bucket_size):max_pwm;

% observation_space
os_space = vertcat(height_space, velocity_space);

% observation space high
os_high = horzcat(max_height, max_vel);

% observation space low
os_low = horzcat(min_height, min_vel);

% observation space window size
os_win_size = (os_high - os_low) ./...
 horzcat(length(height_space), length(velocity_space));

% number of observation space variables
num_os_variables = size(os_space);
num_os_variables = num_os_variables(1);

% learning rate
learning_rate = 0.01;

% discount rate
discount_factor = 0.95;

% number of episodes
episodes = 25000;

% epsilon values (exploration)
epsilon = 0.9;
start_epsilon_decaying = 1;
end_epsilon_decaying = cast((episodes / 2), 'int16');
epsilon_decay_value = 5 * epsilon / ...
    cast(end_epsilon_decaying - start_epsilon_decaying, 'double');

% epsilon_decay_value = 1.01;

% goal height to hit
y_goal = 0.5;

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

%% Q-table initilazation
% low reward value
low_reward = -1;

% randomly initialize q table
q_table = low_reward * rand(length(height_space), ...
    length(velocity_space), length(pwm_space));


%% Simulation
% hold figure so each episode can be plotted together
% hold on

for episode = 1 : 1 : episodes 
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
    
    % set exploration value for chance to explore
        explore = rand;

    for i = 1 : 1 : (length(y_values))
        
        if explore < epsilon
            action = randi([7,length(pwm_space)]);
            current_q = ...
                q_table(discrete_state(1), discrete_state(2), action);
            color = '-r';
        else
            % choose action with highest reward
            [current_q, action] = ...
                max(q_table(discrete_state(1), discrete_state(2), :));
            color = '-g';
        end
        
        % get new simulation step
        [y_current, previous_time_samples, previous_states] = ...
            lsim(G, [pwm_space(action), pwm_space(action)],...
            [0, sampling_rate], previous_state);
        
        % enforce max height
        if y_current(end - 1) > max_height
            y_current(end - 1) = max_height;
            y_values(i) = max_height;
            
        end

        if y_current(end) > max_height
            y_current(end) = max_height;
            y_values(i + 1) = max_height;
        end

        % enforce min height
        if y_current(end - 1) < min_height
            y_current(end - 1) = min_height;
            y_values(i) = min_height;
        end

        if y_current(end) < min_height
            y_current(end) = min_height;
            y_values(i + 1) = min_height;
        end
        
        if y_current == 0 | y_current == max_height
            velocity_current = 0;
        else
            velocity_current = abs(max(calculate_velocity(y_previous,...
            y_current(end), sampling_rate)));
        end
        
        % calculate new discrete step
        new_discrete_state = get_discrete_state([y_current(end),...
            velocity_current], os_low, os_win_size);

        % update previous y value
        y_previous = y_current(end);

        y_difference = abs(y_goal - y_current(end));
        
        % calculate reward
        height_reward_weight = 7;
        height_reward = height_reward_weight * y_difference;
        
        velocity_reward_weight = 1;
        velocity_reward = velocity_reward_weight * abs(velocity_current);
        
        if abs(y_current - y_goal) < 0.07 & velocity_current < 0.1
            reward = 100;
%             disp(['great: ', num2str(action)]);
        elseif abs(y_current - y_goal) < 0.1 & velocity_current < 0.25;
            reward = 5;
%             disp(['good: ', num2str(action)])
        else
            reward = -height_reward;
%             disp(['bad: ', num2str(action)])
        end
        
        % calculate max future q value
        [max_future_q, ~] = max(q_table(new_discrete_state(1)...
            , new_discrete_state(2), :));
        
        % update q value using the bellman equation
        new_q = (1 - learning_rate) * current_q...
            + learning_rate * (reward + discount_factor * max_future_q);
        
        q_table(new_discrete_state(1), ...
            new_discrete_state(2), action) = new_q;
        
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
        y_values(i) = y_current(end - 1);
        y_values(i + 1) = y_current(end);

        % debugging
%         disp(['previous_state: ', num2str(previous_state(end - 1)),...
%             ' ', num2str(previous_state(end))]);
        
%         disp(['episode ', num2str(episode), ':']);
%         disp(['y: ', num2str(y_current(end))]);

%         disp(['velocity: ', num2str(max(calculate_velocity(y_previous,...
%             y_current(end), sampling_rate)))]);

%         disp(['action: ', num2str(2727.0477 + pwm_space(action))]);
%         disp(['action bucket: ', num2str(action)]);
%         disp(['reward: ' num2str(reward)]);
%         disp(['new_q: ', num2str(new_q)]);
%         disp(['epsilon: ', num2str(epsilon)]);
%         fprintf(1, '\n');

    end

    if end_epsilon_decaying >= episode ...
            && episode >= start_epsilon_decaying
        epsilon =  epsilon - epsilon_decay_value;
    end
    
    % logging metrics
    if mod(episode, 10) == 0
        disp(['episode ', num2str(episode), ':']);
        disp(['epsilon: ', num2str(epsilon)]);
        disp(['action: ', num2str(2727.0477 + pwm_space(action))]);
        disp(['reward: ' num2str(reward)]);
        disp(['new_q: ', num2str(new_q)]);
        disp(['y: ', num2str(y_current(end))]);
        disp(['velocity: ', num2str(max(calculate_velocity(y_previous,...
            y_current(end), sampling_rate)))]);
        
        fprintf(1, '\n');
    end
    
    plot(0:sampling_rate:episode_length, y_values, color);
    ylim([0 1])
    xlabel('Time(s)')
    ylabel('Height(m)')
    title('Height vs. Time')
    drawnow
end
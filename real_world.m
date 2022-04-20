% A MATLAB script to control Rowans Systems & Control Floating Ball 
% Apparatus designed by Mario Leone, Karl Dyer and Michelle Frolio. 
% The current control system is a Q table
%
% Created by Zachary Heras 4/5/2022

%% Clear workspace
close all; clc; clear device; clear;

%% Connect to device
device = serialport('COM10', 19200);

%% Q learning parameters
% load q table for desired height
load('real_world_q_table_45cm', 'q_table');

% max height of pipe
max_height = 0.9144;

% min height of pipe
min_height = 0;

% max velocity of ball and pipe system
max_vel = 1;

% min velocity of ball and pipe system
min_vel = -1;

% max acceleration of ball and pipe system
% max_acc = 0.4;

% min acceleration of ball and pipe system
% min_acc = 1.075;

% max pwm of ball and pipe system
max_pwm = 4095;

% min pwm of ball and pipe system
min_pwm = 0;

% bucket size for height
bucket_size_height = 18;

% bucket size for velocity
bucket_size_velocity = 10;

% bucket size for pwm
bucket_size_pwm = 4;

% height space / range
height_space = min_height:...
    ((max_height - min_height) / bucket_size_height):max_height;
 
% velocity space / range
velocity_space = min_vel:((max_vel - min_vel) / bucket_size_velocity):max_vel;

% acceleration space / range
% acceleration_space = min_acc:((max_acc - min_acc) / bucket_size):max_acc;

% pwm space / range
pwm_space = min_pwm:((max_pwm - min_pwm) / bucket_size_pwm):max_pwm;

% observation_space
os_space = zeros(length(height_space), length(velocity_space));

% observation space high
os_high = horzcat(max_height, max_vel);

% observation space low
os_low = horzcat(min_height, min_vel);

% observation space window size
os_win_size = (os_high - os_low) ./...
 horzcat(length(height_space), length(velocity_space));

% number of observation space variables
num_os_variables = length(size(os_space));
num_os_variables = num_os_variables(1);

% time that simulation will run for in seconds
episode_length = 72;

% number of episodes
episodes = 4000;

% amount of time between controller actions
sampling_rate = 0.33;

% number of simulation steps
steps = episode_length / sampling_rate;

% time for fan to switch pwm values
action_time = 0.06;

% learning rate
learning_rate = 0.01;

% discount rate
discount_factor = 0.95;

% epsilon decay function
epsilon = 0.8;
start_epsilon_decaying = 1;
end_epsilon_decaying = cast((episodes / 2), 'int16');
epsilon_decay_value = 0.1 * epsilon / ...
    cast(end_epsilon_decaying - start_epsilon_decaying, 'double');

%% Bring ball to bottom for start
% set fan speed to 0 for 2 seconds
action = 0;
set_pwm(device, action);
pause(2);

%% Initialize variables
% set y_goal
y_goal = 0.4572;

% initialize reward to 0;
reward = 0;

% initialize total reward values for plotting
episode_reward_values = [];

% intialize reward_values
reward_values = [];

%initialize q_table
lowest_starting_q_value = -10;
 
% q_table = lowest_starting_q_value * zeros(length(height_space), ...
%         length(velocity_space), length(pwm_space));

%% Q learning / control
 for episode = 1 : 1 : episodes
    % initialize first discrete state as 0 height and 0 velocity
    discrete_state = get_discrete_state([0, 0], os_low, os_win_size);

    % initilize y_previous to 0
    y_previous = 0;

    % initialize y_current to 0
    y_current = 0;
    
    % initialize vel_previous to 0
    vel_previous = 0;
    
    % initialize vel_current to 0
    vel_current = 0;
    
    % initialize acc_current to 0
%     acc_current = 0;

    % initiazile episode's total reward
    episode_reward = 0;
    
    % initialze y difference previous to 0
    y_difference_current_previous = 0;
    
    % loop through each time step
    for step = 1 : 1 : steps
        
        reward = 0;
        
        % set exploration value for chance to explore
        explore = rand;
    
        % determine if agent will explore
        if explore < 0.0000000011
            % explore
            action = randi([3,length(pwm_space)]);
            current_q = ...
                q_table(discrete_state(1), discrete_state(2), action);
%             disp(['Exploring step ' num2str(step)]);
        else
            % choose action with highest reward
            [current_q, action] = ...
                max(q_table(discrete_state(1), discrete_state(2), :));
%             disp(['action: ' num2str(action)]);
        end

        % take action
        set_pwm(device, pwm_space(action));
        
        % wait for fan to react to action
%         pause(action_time);
        
        % wait for sample
        pause(sampling_rate);
        
        % get new simulation step
        y_current = read_data(device);
        y_current = ir2y(y_current);
        
        % calculate velocity
        vel_current = (y_current - y_previous) / sampling_rate;
    
        % calculate acceleration
%         acc_current = (vel_current - vel_previous) / sampling_rate;

        % calculate new discrete step
        new_discrete_state = get_discrete_state([y_current,...
            vel_current], os_low, os_win_size);

        % update previous y value
        y_previous = y_current;

        % update previous velocity value
        vel_previous = vel_current;
        
        % calculate difference
        y_difference_current = y_current - y_goal;

        % simulation reward function
        height_reward_weight = 10;
        height_reward = height_reward_weight * abs(y_difference_current);

        velocity_reward_weight = 1;
        velocity_reward = velocity_reward_weight * abs(vel_current);

        if (y_difference_current < 0.05 & y_difference_current > 0) & (vel_current > -0.3 & vel_current < 0) %#ok<AND2>
            reward = 55;
        elseif (y_difference_current < 0.05 & y_difference_current > 0) & (vel_current < 0.3 & vel_current > 0) %#ok<AND2>
            reward = 55;
        end
        
        reward = reward -(height_reward)^2 + 15;

        % update previous y difference
        y_difference_current_previous = y_current;
        
        % add single step reward to total reward 
        episode_reward = episode_reward + reward;

        % calculate max future q value
        [max_future_q, ~] = max(q_table(new_discrete_state(1)...
            , new_discrete_state(2), :));

        % update q value using the bellman equation
        new_q = (1 - learning_rate) * current_q...
            + learning_rate * (reward + discount_factor * max_future_q);

        q_table(discrete_state(1), ...
            discrete_state(2), action) = new_q;

        % update discrete_state
        discrete_state = new_discrete_state;

        % update epsilon
        if end_epsilon_decaying >= episode ...
                && episode >= start_epsilon_decaying
            epsilon =  epsilon - epsilon_decay_value;
        end
        
%         figure(1)
%         reward_values(end + 1) = reward;
%         plot(1:1:length(reward_values), reward_values)
%         xlabel('Sample')
%         ylabel('Reward')
%         title('Reward vs. Sample')
%         drawnow
        
    end
    
    % bring ball back to bottom
    set_pwm(device, min_height);
    
    % update y_values and reward_values for plotting
    episode_reward_values(end + 1) = episode_reward;
    
    % plot total episode rewards
    figure(2)
    plot(1:1:length(episode_reward_values), episode_reward_values);
    xlabel('episodes');
    ylabel('reward');
    title('Reward vs. Episodes')
    
    % save total episode rewards over time
    save('episode_reward_values', 'episode_reward_values')
    
    % logging
    disp(['episode: ', num2str(episode)])
    disp(['episode_episode_reward: ', num2str(episode_reward)]);
    fprintf(1, '\n');
    
    % save updated q table
    y_goal_cm = y_goal * 100;
    save(['real_world_q_table_' num2str(y_goal_cm) 'cm'], 'q_table');
    
    pause(2);
 end
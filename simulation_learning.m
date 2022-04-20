% A MATLAB script to train a q table for use as a controller
% Created by Zachary Heras 2/23/2022

%% Clear workspace and command window
clear, clc;

%% Given variables in SI units
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
sampling_rate = 0.3;

% max height of pipe
max_height = 0.9144;

% max velocity of ball and pipe system
max_vel = 1;

% min velocity of ball and pipe system
min_vel = -1;

% max pwm of ball and pipe system
max_pwm = 4095 - 2727.0477;

% min pwm of ball and pip system
min_pwm = 0 - 2727.0477;

%% hyperparameters
% time that simulation will run for in seconds
episode_length = 18;

% number of simulation steps
steps = cast(cast((episode_length / sampling_rate), 'int16'), 'double');

% bucket size for height
bucket_size_height = 18;

% bucket size for velocity
bucket_size_velocity = 10;

% bucket size for pwm
bucket_size_pwm = 5;

% min height of pipe
min_height = 0;

% height space / range
height_space = min_height:...
    ((max_height - min_height) / bucket_size_height):max_height;
 
% velocity space / range
velocity_space = min_vel:((max_vel - min_vel) / bucket_size_velocity):max_vel;

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

% number of episodes
episodes = 4000;

% learning rate
learning_rate = 0.01;

% discount rate
discount_factor = 0.90;

% epsilon decay function
epsilon = 0.5;
start_epsilon_decaying = 1;
end_epsilon_decaying = cast((episodes / 2), 'int16');
epsilon_decay_value = 0.04 * epsilon / ...
    cast(end_epsilon_decaying - start_epsilon_decaying, 'double');


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

%% Simulation and Q-learning
% hold figure so each episode can be plotted together
% hold on

% initialize height goal
y_goal = 0.4572;

% low reward value
low_reward = -1;

% initialze reward min
reward_min = [4000 4000 4000 4000 4000 4000 4000 4000 4000];

% while y_goal < max_height
    
    % initialize epsilon
%     epsilon = 0.9;
    
    % randomly initialize q table
    q_table = low_reward * zeros(length(height_space), ...
        length(velocity_space), length(pwm_space));
    
    % initialize previous total episode reward
    total_episode_reward_previous = 0;
    
    % initialize indentical q to 0
    identical_q = 0;
    
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
            
        % episode's total reward
        total_episode_reward = 0;

        % loop through each time step in each episode
        for time = 1 : 1 : (length(y_values))

            % determine if agent will explore
            if explore < epsilon
                action = randi([1,length(pwm_space)]);
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
tic
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
            
            % calculate difference
            y_difference = y_current(end) - y_goal;

            % calculate reward
            height_reward_weight = 40;
            height_reward = height_reward_weight * abs(y_difference);

            velocity_reward_weight = 1;
            velocity_reward = velocity_reward_weight * abs(velocity_current);

            if (y_difference < 0.05 & y_difference > 0) & (velocity_current > -0.26 & velocity_current < 0) %#ok<AND2>
                reward = 50;
            elseif (y_difference < 0.05 & y_difference > 0) & (velocity_current < 0.26 & velocity_current > 0) %#ok<AND2>
                reward = 50;
            else
                reward = -height_reward;
            end
            
            % reward max velocity
%             if(y_current > 0.25 & y_current < 0.65 & abs(velocity_current) > 0.5) %#ok<AND2>
%                 reward = 50;
%             else
%                 reward = -10;
%             end

            % episode's total reward
            total_episode_reward = total_episode_reward + reward;
            
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

            if end_epsilon_decaying >= episode ...
                    && episode >= start_epsilon_decaying
                epsilon =  epsilon - epsilon_decay_value;
            end
toc
        end

        % logging metrics
        if mod(episode, 30) == 0
            disp(['episode ', num2str(episode), ':']);
            disp(['epsilon: ', num2str(epsilon)]);
            disp(['action: ', num2str(2727.0477 + pwm_space(action))]);
            disp(['reward: ' num2str(reward)]);
            disp(['new_q: ', num2str(new_q)]);
            disp(['y difference: ', num2str(y_difference(end))]);
            disp(['velocity: ', num2str(max(calculate_velocity(y_previous,...
                y_current(end), sampling_rate)))]);
            disp(['total episode reward: ', num2str(total_episode_reward)])
            fprintf(1, '\n');

        end

        % visualise agent learning
        figure(2)
        plot(0:sampling_rate:episode_length, y_values, color);
        ylim([0 1])
        xlabel('Time(s)')
        ylabel('Height(m)')
        title('Height vs. Time')
        drawnow
        
        % count identical runs
        if total_episode_reward == total_episode_reward_previous
            identical_q = identical_q + 1;
        end
        
        % end agent of total reward is very high
        % if total_episode_reward > 5000
        %    disp(['total episode reward: '...
        %        , num2str(total_episode_reward)]);
        %    fprintf(1, '\n');
        %    pause(5);
        %    break;
        % end
        
        % end agent if it gets stuck in loop
        if identical_q > 300
            disp(['total episode reward: '...
                , num2str(total_episode_reward)]);
            fprintf(1, '\n');
            pause(5);
            break;
        end
        
        % add single step reward to total reward 
        total_episode_reward_previous = total_episode_reward;
        
    end
    
    y_goal_cm = y_goal*100;
    y_goal_index = cast(y_goal_cm/10, 'int8');
    
    % check if q table is good enough for use
    if total_episode_reward < reward_min(y_goal_index)
        disp(['q_table_'  num2str(y_goal_cm)  'cm' ' not good enough!']);
        disp('redo!');
        fprintf(1, '\n');
        y_goal = y_goal - 0.1;
    else
        disp(['q_table_'  num2str(y_goal_cm)  'cm ' 'saved!']);
        disp('moving to next target height!');
        fprintf(1, '\n');
    end
    
    % save q table
 	save(['q_tables\q_table_'  num2str(y_goal_cm)  'cm'], 'q_table');
%     save(['q_tables\q_table_'  'max_velocity'], 'q_table');
    
    % increment goal height
    y_goal = y_goal + 0.1;
    
% end
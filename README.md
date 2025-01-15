# Ball and Pipe Q-learning Control System
## Overview of Project
This project implements a Q-learning control system for a ball and pipe system that hovers a ball in the air. The control system is implemented in MATLAB and takes advantage of MATLAB's ability to easily create and modify matrices. The ball and pipe system is made of a hollow transparent vertical cylinder with a ping pong ball that sits inside of it. At the bottom of the pipe there is a fan whose RPM / PWM can be controlled by MATLAB via serial communication over USB. The height of the ball is tracked using an IR sensor at the top of the pipe. The goal of the project was to get the ball to float halfway up the tube (43cm high) using a control system. The control system chosen for this project was Q-learning. This control system was chosen because our group wanted to learn about a basic reinforcement learning algorithm. A Q-learning algorithm was implemented to keep the ball hovering at 43cm in the pipe and in the end the algorithm did a decent job of finding a policy to achieve this goal. This is shown in the GIF below.

![](https://github.com/Ryan-Oliver-2k/ball_and_pipe_control/blob/main/ball_and_pipe_system_q_learning_episode_600.gif)

## Summary of Q-Learning
Q-learning is a model-free type of reinforcement learning that houses Q-values to iteratively improve the action selection policy for any given Markov decision process. The Q in Q-learning stands for the “quality” of the action selection policy/function that the algorithms computes. We want the Q-function that maximizes the reward in the given environment, with a given goal in mind. The Q-function in this case is a lookup table called a Q-table with a given Q-value for each action given the height and velocity. The Q-learning algorithm will iteratively learn this optimal Q-function by using the Bellman Optimality Equation. This equation is seen below.

![](https://github.com/Ryan-Oliver-2k/ball_and_pipe_control/blob/main/bellman_optimality_equation.png)

This equation updates the Q-table reward value (Q-value) for a given state and action. The hyperparameters of this equation are α which is the learning rate, (controls how fast the algorithm will converge), and ɣ which is the discount factor (controls how much the algorithm cares about the future reward). Over time the algorithm will converge and will find the actions that maximize the reward for the environment.
 
For this project, the function that the algorithm is optimizing is the action (fan RPM/PWM) that the ball and pipe system controller should take in order to hover the ball in the middle (43cm high) of the pipe given the height and velocity of the ball. In this project, the action space is the varying PWM speeds of the fan, and the observation space is the varying heights and velocities of the ball. The action space is divided into three bins, and the observation space is divided into ten bins. These are hyperparameters that can be changed in the code, but we found that these values worked best for us. The reward function is shown in the code. Basically, it rewards the Q-learning agent if the ball is close to the target. Once the algorithm is set up for the environment, it runs until it reaches an optimal Q-table. This Q-table can then be used as a control system that tells the fan what action to take in order to hover the ball halfway up the tube.

## Code Use Guide
To use the code, make a clone of the repo and run the real_world.m MATLAB script when the ball and pipe system USB electrical connection is plugged into the computer running the script. For this script to work, make sure to set your device COM port to the correctly. Currently, there is a Q-table that was able to be used as a control system to hover the ball in the middle as shown in the overview of project section. However, this table does not work consistently because the ball and pipe system is not very secure and the angle at which it is facing up changes. 
Below are brief descriptions of what each file and directory does/is in this repository:
- real_world_figures: Contains real world training reward values over time.
- real_world_q_tables: Contains q tables that were trained on the actual ball and pipe environment.
- calculate_velocity.m: Calculates the velocity of the ball given a current and previous height.
- get_discrete_state.m: Calculates the discrete state for a given continuous state.
- ir2y.m: Converts an IR sensor reading into an actual height in meters.
- partial_success_episode_513.fig: Figure showing the reward graph for 513 episodes.
- read_data.m: MATLAB script that reads height data from the ball and pipe system.
- set_pwm.m: MATLAB script that sets the fan PWM for the ball and pipe system.
- real_world.m: Runs a given Q-table on the ball and pipe system and continues to optimize it.
- simulation_learning: Shows the Q-learning algorithm working in real time on a simulation of the ball and pipe system. This is a standalone file.
- SCFBA Specfication Sheet.pdf: Datasheet for the ball and pipe system.

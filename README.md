# Ball and Pipe Control System 

## Overview of Project
This project uses MATLAB code to open serial communication with a ball and pipe system.
The system is made of a vertical cylinder with a ping pong ball controlled 
by a fan on the bottom and height measured by a time of flight sensor on top. 
The objective is to balance the ball at a target altitude. To achieve this, we used 
Q-learning to have the system reach a target altitude through reinforcement training. 
Using Q-learning and reinforcement learning for this type of project allows for a model-free,
discrete action system reach a set value wihtout needing any other adaptations outside of 
setting that value. Given enough trials, it efficiently optimizes for the best actions needed 
to reach the target. 
Some problems that we experienced throughout this project was getting MATLAB to work with 
the serial port correctly, and to correctly transmit the values of each state to the ball 
height. Throughout setting up the ball and pipe system, the ball height would keep, 
inexplicably, going to the max height of the system and stay there in spite of other inputs.
Through trial and error with different variables, and setting the pulse width modulated signal
to different values, this was eventually able to be resolved and we were able to start training.

## Summary of Q-Learning
Q-Learning is a model-free type of reinforcement learning that uses Q-values to iteratively improve the learning model, 
where the Q-value is basically an estimate of how good a certain action is at a given state. In order to use Q-Learning, first
something called a Q-table is made. This table is a matrix of all states and current actions on the model and starts out at
all zeroes. After each episode, or basically when the agent reaches a terminating state where no more transitions of state are possible,
the Q-value for that run is stored in that matrix. This then becomes the table our agent references in order to 
pick the best action based on its Q-value. Over time, this agent will converge on the optimal Q-values that lead to the highest reward.
To interact with the environment, the agent can either explore of exploit. When the agent explores, actions are taken at random and their 
values recorded. When the agent exploits, actions are taken based on the highest reward using the Q-table as reference.
So, the how Q-Learning basically works is an agent in a state takes an action and receives a reward, 
decides a next action by either referencing the Q-table or at random, and then updates the Q-values accordingly.

## Code Use Guide
In order to use the code, you must download the entire repo, and run the real_world.m script. For this script to work, make sure to set your device COM port directly by finding the com port for the B&P system in your device manager. When you change the com port, keep the baud rate to 19200, as that is the baud rate for the device.

Currently, a q-table exists to get the ball to reach a pipe height of 43cm, and hover there. In order to reach other heights, the data can be trained with different parameters for the height.

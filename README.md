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

## Summary of Q-Learning


## Code Use Guide

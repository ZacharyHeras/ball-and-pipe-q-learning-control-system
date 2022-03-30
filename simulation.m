%% Created by Zachary Heras 2/23/2022
clear, clc;

%% Given Variables
% mass = 0.00270kg
% volume = 0.000268m^3
% density = 1.22500kg/m^3
% g = 9.8
% velocity_eq = 2.4384m/s

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

%% G1: Wind speed to ball position
syms s;
c2 = ((2*g) / velocity_eq) * (mass - density * 0.00026) / (mass);
Ys = c2; % Just reassigned for nomenclature
Vs = sym2poly(s*(s+c2));
G1 = tf(Ys,Vs);

%% G2: PWM to air speed
G2 = 6.3787 * 10^-4;

%% G3: PWM to Ball Position
G3 = G2 * G1;


%% Checking simulation for accuracy
t = 0:0.25:3;
t2 = 0:0.25:6.25;
u = 4095 * ones(length(t), 1);
v = 0 * ones(length(t), 1);
z = vertcat(u, v);

length(z)

lsim(G3, z, t2);

ylim([-0.5 6.25])
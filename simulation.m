%% Given Variables
% mass = 0.00270kg
% volume = 0.000268m^3
% density = 1.22500kg/m^3
% g = 9.8
% velocity_eq = 2.4384m/s
% 
% Created by Zachary Heras 2/23/2022

% mass of ping pong ball
mass = 0.00270;

% volume of ping pong ball
volume = 0.00026;

% density of ping pong ball
density = 1.22500;

% acceleration of gravity
g = 9.8;

% wind velocity when ball is in equilibrium
% this is an approximate value, real value changes with height
velocity_eq = 2.4384;

%% Ball position to wind speed transfer function G1
syms s;
c2 = (2*g)/velocity_eq *(mass - density*0.00026)/(mass);
Ys = c2;
Vs = sym2poly(s*(s+c2));
G1 = tf(Ys,Vs);

%% PWM to air speed G2
G2 = 6.3787 * 10^-4;

%% PWM to Ball Position Cascaded Transfer Function G3
G3 = G2 * G1;

G4 = ss(G3);
disp(G4.A);
disp(G4.B);
disp(G4.C);
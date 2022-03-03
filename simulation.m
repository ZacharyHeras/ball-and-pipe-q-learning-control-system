%% Given Variables
% mass = 0.00270kg
% volume = 0.000268m^3
% density = 1.22500kg/m^3
% g = 9.8
% velocity_eq = ??
% 
% Created by Zachary Heras 2/23/2022

mass = 0.00270;
volume = 0.00026;
density = 1.22500;
g = 9.8;
velocity_eq = 0; % This is a placeholder

%% Ball Position to Wind Speed Transfer Function
syms s;
c2 = (2*g)/veq *(mass - density*0.00026)\(mass);
Ys = sym2poly(c2);
Vs = sym2poly(s*(s+c2));
G1 = tf(Ys,Vs);

%% 

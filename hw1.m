% 1st Homework - Flight Control
% Analysis of dynamic aircraft behavior and design of a stability
% augmentation system

clearvars; close all; clc

global plot_colors 
plot_colors = [55, 126, 184; ...
              228,  26,  28; ...
               77, 175,  74; ...
              152,  78, 163; ...
              255, 127,   0; ...
              255, 255,  51]/255;

% Set up the task
Vr = 180/1.944; % m/s (180 KTAS)
H = 4000/3.281; % m (4000 ft)
g = 9.81;

C = eye(4); D = 0;

% longitudinal motion
A_long = [-.9981 -2.5072 -3.518e-4 0;
         .9709 -.9632 -.0025 .0099;
         -.1274 4.6526 -.0219 -9.7234;
         1 0 0 0];

B_long = [.1335 -5.6897;
         -.0048 -.1038;
         2.9885 -.6188;
         0 .0518];

sysFull_long = ss(A_long,B_long,C,D);
sysFull_long.StateName = {'q','\alpha','V','\Theta'};
sysFull_long.InputName = {'\delta_t','\delta_e'};
sysFull_long.OutputName = sysFull_long.StateName;
sysFull_long.TimeUnit = 'seconds';

% lateral motion
A_lat = [-.4956 1.9243 -.1206 0;
         -.9795 -.193 .0963 .1055;
         2.107 -5.5049 -3.3388 0;
         0 0 1 0];

B_lat = [-.4515 -1.318;
         0 .0362;
         -9.498 1.9929;
         0 0];

sysFull_lat = ss(A_lat,B_lat,C,D);
sysFull_lat.StateName = {'r','\beta','p','\Phi'};
sysFull_lat.InputName = {'\delta_a','\delta_r'};
sysFull_lat.OutputName = sysFull_lat.StateName;
sysFull_lat.TimeUnit = 'seconds';


%% Task 1: Analysis of the eigenbehavior
task1


%% Task 2: Design of a flight controller with damper support in the longitudinal motion (SAS)
%task2


%% Task 3: Design of a base controller for ecoupling lateral aircraft movement (SAS)
%task3


%% Task 4: Testing the controllers in the nonlinear simulation
%task4
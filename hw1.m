% 1st Homework - Flight Control
% Analysis of dynamic aircraft behavior and design of a stability
% augmentation system

clearvars; close all; clc

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

%% Task 1: Analysis of the eigenbehavior

% Short-period
A_sp_approx = A_long(1:2,1:2);
A_sp_approx = flip(flip(A_sp_approx,2),1);
A_sp_approx(1,2) = 1; % approximation for steady horizontal flight without wind

B_sp_approx = B_long(1:2,:);
B_sp_approx = flip(B_sp_approx,1);

sp_approx = ss(A_sp_approx,B_sp_approx,eye(2),0);
sp_approx.StateName = {'\alpha','q'};
sp_approx.InputName = {'\delta_t','\delta_e'};
sp_approx.OutputName = sp_approx.StateName;

% Natural frequency and damping
[Wn_sp,Z_sp] = damp(sp_approx);

% Step input
delta_e_max = deg2rad(20);

[Y_sp_approx,T] = step(delta_e_max*sp_approx,10);
%figure(1); plot(T,Y_sp_approx(:,1),T,Y_sp_approx(:,2));grid on
%legend({'$\alpha$','$q$'},'Interpreter','latex','Location','best','FontSize',14)



% Dutch roll
A_dr_approx = A_lat(1:2,1:2);
B_dr_approx = B_lat(1:2,:);

dr_approx = ss(A_dr_approx,B_dr_approx,eye(2),0);
dr_approx.StateName = {'r','\beta'};
dr_approx.InputName = {'\delta_a','\delta_r'};
dr_approx.OutputName = dr_approx.StateName;

delta_a_max = deg2rad(20);
delta_r_max = deg2rad(20);

[Y_dr_approx,T] = step(sp_approx,10);
%figure(2); plot(T,Y_dr_approx(:,1),T,Y_dr_approx(:,2));grid on
%legend({'$r$','$\beta$'},'Interpreter','latex','Location','best','FontSize',14)
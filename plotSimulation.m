%% Plots from simulation
%% Longitudinal motion without controller
alt = load('Simulation Data\sw_00370004_TUB614_0501.mat');

t = alt.data.wd_time;
q = alt.data.wd_qb; % qsim is not right
alpha = alt.data.wd_al; % alpha?
v = alt.data.wd_ve;
theta = alt.data.wd_theta;

subplot(4,1,1)
plot(t, q)
ylabel('q')
grid on
title('Longitudinal motion without controller')

subplot(4,1,2)
plot(t, alpha)
ylabel('\alpha')
grid on

subplot(4,1,3)
plot(t, v)
ylabel('V')
grid on

subplot(4,1,4)
plot(t, theta)
ylabel('\theta')
xlabel('time[s]')
grid on

%% Longitudinal motion with controller
alt = load('Simulation Data\sw_00370004_TUB614_0503.mat');

t = alt.data.wd_time;
q = alt.data.wd_qb; % qsim is not right
alpha = alt.data.wd_al; % alpha?
v = alt.data.wd_ve;
theta = alt.data.wd_theta;

subplot(4,1,1)
plot(t, q)
ylabel('q')
grid on
title('Longitudinal motion with controller')

subplot(4,1,2)
plot(t, alpha)
ylabel('\alpha')
grid on

subplot(4,1,3)
plot(t, v)
ylabel('V')
grid on

subplot(4,1,4)
plot(t, theta)
ylabel('\theta')
xlabel('time[s]')
grid on



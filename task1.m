% Task 1: Analysis of the eigenbehavior

%% 1.1
fprintf('Flying Qualities:\n')

% Longitudinal Motion
Tsim_long = 10;
% Step input
delta_e_max = deg2rad(20); % elevator deflection of 20°

% Full system
[Y_Full_long1,T_Full_long1] = step(delta_e_max*sysFull_long(:,2),Tsim_long);
Y_Full_long1(:,[1 2 4]) = rad2deg(Y_Full_long1(:,[1 2 4]));

plotStepResponse(T_Full_long1,Y_Full_long1(:,1:2),"full","SP");

% Approximation Short-Period
sp_approx = approximateSystem(A_long,B_long,"SP");

[Y_sp_approx,T_sp_approx] = step(delta_e_max*sp_approx(:,2),Tsim_long);
Y_sp_approx = rad2deg(Y_sp_approx);

plotStepResponse(T_sp_approx,Y_sp_approx,"approximation","SP");

% Natural frequency and damping
[Wn_sp,Z_sp] = damp(sp_approx);

% Control Anticipation Parameter (CAP)
alphad_ss = dcgain(delta_e_max*tf([1 0],[1])*sp_approx(1,2)); % steady-state response of \dot{\alpha}
q_ss = dcgain(delta_e_max*sp_approx(2,2)); % steady-state response of q
nz_ss = Vr*(q_ss-alphad_ss)/g;
%[NUM,DEN] = tfdata(delta_e_max*tf([1],[1 0])*tf([1 0 0],[1])*sp_approx(2,2),'v');
%qd_init = NUM(1)/DEN(1);
%CAP = qd_init/nz_ss;
%or
alpha_ss = dcgain(delta_e_max*sp_approx(1,2));
ratio_nzalpha = nz_ss/alpha_ss;
CAP = Wn_sp(1)^2/ratio_nzalpha;

fprintf('Short-period:\n');
fprintf('natural frequency: %.3f\ndamping ratio: %.3f\n',Wn_sp(1),Z_sp(1));
fprintf(['- Fingerprint plot: poor behavior\n', ...
         '- Damping ratio limits (MIL-F-8785 C): level 1\n', ...
         '- CAP-factor: level 1\n\n']);


% Lateral-Directional Motion
Tsim_lat = 15;
% Step input
delta_r_max = deg2rad(20); % rudder deflection of 20°

% Full system
[Y_Full_lat,T_Full_lat] = step(delta_r_max*sysFull_lat(1:2,2),Tsim_lat);
Y_Full_lat = rad2deg(Y_Full_lat);

plotStepResponse(T_Full_lat,Y_Full_lat,"full","DR");

% Approximation Dutch Roll
dr_approx = approximateSystem(A_lat,B_lat,'DR');

[Y_dr_approx,T_dr_approx] = step(delta_r_max*dr_approx(:,2),Tsim_lat);
Y_dr_approx = rad2deg(Y_dr_approx);

plotStepResponse(T_dr_approx,Y_dr_approx,"approximation","DR");

% Natural frequency and damping
[Wn_dr,Z_dr] = damp(dr_approx);

fprintf('Dutch roll:\n');
fprintf('natural frequency: %.3f\ndamping ratio: %.3f\n',Wn_dr(1),Z_dr(1));
fprintf(['- Minimum dutch roll frequency and damping (MIL-F-8785 C): level 1\n\n']);

saveas(figure(1),'./figures/task1.1-longitudinal-comparison.png');
saveas(figure(2),'./figures/task1.1-lateral-comparison.png');

%% 1.3
% Step input
delta_t_max = .1; % 10% throttle
[Y_Full_long2,T_Full_long2] = step(delta_t_max*sysFull_long(:,1),Tsim_long);
Y_Full_long2(:,[1 2 4]) = rad2deg(Y_Full_long2(:,[1 2 4]));

figure(3); clf
label_y = ["$q [^\circ/s]$","$\alpha [^\circ]$","$V [m/s]$","$\Theta [^\circ]$"];
subplot_pos = 0;
for i = 1:4
    subplot_pos = subplot_pos + 1;
    subplot(4,2,subplot_pos); grid on; hold all
    ylabel(label_y(i),'Interpreter','latex','FontSize',12);
    plot(T_Full_long2,Y_Full_long2(:,i),'LineWidth',1.5,'Color',plot_colors(1,:));
    if i == 1
        title('$\delta_t = 10\%$', ...
              'Interpreter','latex', ...
              'FontSize',16);
    elseif i == 4
        xlabel('$t [s]$','Interpreter','latex','FontSize',12);
    end

    subplot_pos = subplot_pos + 1;
    subplot(4,2,subplot_pos); grid on; hold all
    plot(T_Full_long1,Y_Full_long1(:,i),'LineWidth',1.5,'Color',plot_colors(1,:));
    if i == 1
        title('$\delta_e = 20^\circ$', ...
              'Interpreter','latex', ...
              'FontSize',16);
    elseif i == 4
        xlabel('$t [s]$','Interpreter','latex','FontSize',12);
    end
end
hold off
saveas(figure(3),'./figures/task1.3-thrust-elevator-comparison.png');
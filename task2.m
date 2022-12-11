% Task 2: Design of a flight controller with damper support in the longitudinal motion (SAS)

%% 2.2 Only for testing purpose
Z_sp_desired = .7; % desired short-period mode damping ratio

F_qdeltae = tf(sp_approx(1,2));

fprintf("For the standard negative feedback, the root locus shows that the system goes unstable" + ...
        " with a small value of K and\nthe closed loop system cannot achieve a damping ratio of 0.7.\n" + ...
        "Hence, the positive feedback is to be considered.\n\n");

%figure(4); rlocus(-F_qdeltae); set(findall(gca,'Type','Line'),'LineWidth',1.5,'MarkerSize',10); grid on
K_rlocus = .1683;

sp_approx_rlocus = feedback(sp_approx,K_rlocus,2,1,1);
[Y_sp_approx_rlocus, T_new] = step(delta_e_max*sp_approx_rlocus(:,2),Tsim_long);
Y_sp_approx_rlocus = rad2deg(Y_sp_approx_rlocus);

% Plot step response
figure(4);
label_y = ["$q [^\circ/s]$","$\alpha [^\circ]$"];
for i = 1:2
    subplot(2,1,i); hold all; ylabel(label_y(i),'Interpreter','latex','FontSize',12);
    plot(T,Y_sp_approx(:,i),'LineWidth',1.5,'Color',plot_colors(1,:),'LineStyle','--');
    plot(T_new,Y_sp_approx_rlocus(:,i),'LineWidth',1.5,'Color',plot_colors(1,:),'LineStyle','-');
end
hold off
grid on
xlabel('$t [s]$','Interpreter','latex','FontSize',12);
subplot(2,1,2); lgd = legend('Short-period  approximation','feedback controller $q \rightarrow \delta_e$');
lgd.Location = 'northeast'; lgd.FontSize = 12;
lgd.Interpreter = 'latex'; lgd.NumColumns = 2;
subplot(2,1,1); title('Elevator deflection $\delta_e$ of $20^\circ$', ...
                      'Interpreter','latex', 'FontSize',14);

saveas(figure(4),'./figures/task2.2-original-rlocus-comparison.png');

%% 2.3
Z_sp_desired = .7; % desired short-period mode damping ratio
Wn_sp_desired = 2*pi*.4775; % desired short-period mode natural frequency
pole = -Z_sp_desired*Wn_sp_desired+1j*Wn_sp_desired*sqrt(1-Z_sp_desired^2);
desiredPoles = [pole conj(pole)];

Qc = ctrb(sp_approx.A,sp_approx.B(:,2));    % controllability matrix
rank(Qc)

% Pole placement
K_place = place(sp_approx.A,sp_approx.B(:,2),desiredPoles);
sp_approx_place = feedback(sp_approx,K,2,[1 2],-1);
sysFull_long_place = feedback(sysFull_long,K,2,[1 2],-1);

damp(sp_approx_place)
damp(sysFull_long_place)


%% 2.4
Tsim_long = 10;
% original
[Y_Full_long2,T2] = step(delta_e_max*sysFull_long(:,2),Tsim_long);
Y_Full_long2(:,[1 2 4]) = rad2deg(Y_Full_long2(:,[1 2 4]));

% root locus
sysFull_long_rlocus = feedback(sysFull_long,K_rlocus,2,1,1);
[Y_Full_long_rlocus,T_rlocus] = step(delta_e_max*sysFull_long_rlocus(:,2),Tsim_long);
Y_Full_long_rlocus(:,[1 2 4]) = rad2deg(Y_Full_long_rlocus(:,[1 2 4]));

% pole placement
[Y_Full_long_place,T_place] = step(delta_e_max*sysFull_long_place(:,2),Tsim_long);
Y_Full_long_place(:,[1 2 4]) = rad2deg(Y_Full_long_place(:,[1 2 4]));

figure(5); clf;
label_y = ["$q [^\circ/s]$","$\alpha [^\circ]$","$V [m/s]$","$\Theta [^\circ]$"];
for i = 1:4
    subplot(4,1,i); hold all; ylabel(label_y(i),'Interpreter','latex','FontSize',12);
    plot(T2,Y_Full_long2(:,i),'LineWidth',1.5,'Color',plot_colors(1,:));
    plot(T_rlocus,Y_Full_long_rlocus(:,i),'LineWidth',1.5,'Color',plot_colors(1,:),'LineStyle','--');
    plot(T_place,Y_Full_long_place(:,i),'LineWidth',1.5,'Color',plot_colors(1,:),'LineStyle','-.');
end
hold off
grid on
xlabel('$t [s]$','Interpreter','latex','FontSize',12);
subplot(4,1,3); lgd = legend('original','root locus','pole placement');
 lgd.Location = 'northwest'; lgd.FontSize = 11;
lgd.Interpreter = 'latex'; lgd.NumColumns = 1;
subplot(4,1,1); title('Elevator deflection $\delta_e$ of $20^\circ$', ...
                      'Interpreter','latex', 'FontSize',14);

saveas(figure(5),'./figures/task2.4-original-rlocus-place-comparison.png');

%% 2.5
Z_ph_desired = .707; % desired short-period mode damping ratio

F_Thetadeltae = tf(sysFull_long_place(4,2));
figure(6); rlocus(-F_Thetadeltae);set(findall(gca,'Type','Line'),'LineWidth',1.5,'MarkerSize',10);
K_ph = .355;

sysFull_long_new = feedback(sysFull_long_place,K_ph,2,4,1);

Tsim_long = 300;
[Y_Full_long_place,T_place] = step(delta_e_max*sysFull_long_place(:,2),Tsim_long);
Y_Full_long_place(:,[1 2 4]) = rad2deg(Y_Full_long_place(:,[1 2 4]));

[Y_Full_long_new,T_new] = step(delta_e_max*sysFull_long_new(:,2),Tsim_long);
Y_Full_long_new(:,[1 2 4]) = rad2deg(Y_Full_long_new(:,[1 2 4]));

%
% F_Vdeltat = tf(sysFull_long_place(3,1));
% K_ph = .0386;
% 
% sysFull_long_new2 = feedback(sysFull_long_place,K_ph,1,3,-1);
% [Y_Full_long_new2,T_new2] = step(delta_e_max*sysFull_long_new2(:,2),Tsim_long);
% Y_Full_long_new2(:,[1 2 4]) = rad2deg(Y_Full_long_new2(:,[1 2 4]));
% 
% u = K_ph*Y_Full_long_new2(:,3);
%


figure(6); clf;
label_y = ["$q [^\circ/s]$","$\alpha [^\circ]$","$V [m/s]$","$\Theta [^\circ]$"];
for i = 1:4
    subplot(4,1,i); hold all; ylabel(label_y(i),'Interpreter','latex','FontSize',12);
    plot(T_new,Y_Full_long_new(:,i),'LineWidth',1.5,'Color',plot_colors(1,:),'LineStyle','-');
    plot(T_new2,Y_Full_long_new2(:,i),'LineWidth',1.5,'Color',plot_colors(1,:),'LineStyle','-.')
    plot(T_place,Y_Full_long_place(:,i),'LineWidth',1.5,'Color',plot_colors(1,:),'LineStyle','--');
end
hold off
grid on
xlabel('$t [s]$','Interpreter','latex','FontSize',12);
subplot(4,1,1); lgd = legend('with phugoid damping','without phugoid damping');
lgd.Location = 'southeast'; lgd.FontSize = 11;
lgd.Interpreter = 'latex'; lgd.NumColumns = 1;
title('Elevator deflection $\delta_e$ of $20^\circ$', ...
      'Interpreter','latex', 'FontSize',14);

saveas(figure(6),'./figures/task2.6-with-out-phugoid-damping-comparison.png');
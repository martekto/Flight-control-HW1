%% Task 3: Design of a base controller for decoupling lateral aircraft movement (SAS)

%% 3.2

Qc = ctrb(A_lat,B_lat);    % controllability matrix
rank(Qc);


%% 3.3

desiredPoles = [-.7143, -.15+.477j, -.15-.477j, .0347]; % Roll, Dutch roll, Spiral


%% 3.4

% Eigenstructure assignment: Roll
v_1d = [1 0 1 1]'; % [* 0 1 *]'
D1 = [eye(2),zeros(size(A_lat)-2)];
v1_tilde = D1*v_1d;

lambda1 = desiredPoles(1);
M1 = [lambda1*eye(size(A_lat))-A_lat B_lat;D1 zeros(size(D1,1),size(B_lat,2))];
b1 = [zeros(size(A_lat,1),1);v1_tilde];

v1 = M1\b1; u1 = v1(5:6); v1 = v1(1:4);

% Eigenstructure assignment: Dutch roll
v_2d = [1 1 0 1]'; % [* 1 0 *]'
D2 = [0 1 0 0;0 0 1 0];
v2_tilde = D2*v_2d;

lambda2 = desiredPoles(2);
M2 = [lambda2*eye(size(A_lat))-A_lat B_lat;D2 zeros(size(D2,1),size(B_lat,2))];
b2 = [zeros(size(A_lat,1),1);v2_tilde];

v2 = M2\b2;
u2 = v2(5:6); v2 = v2(1:4);
u3 = conj(u2); v3 = conj(v2);


% Eigenstructure assignment: Spiral
v_4d = [1 0 1 1]'; % [* 0 * 1]'
D4 = [0 1 0 0;0 0 0 1];
v4_tilde = D4*v_4d;

lambda4 = desiredPoles(4);
M4 = [lambda4*eye(size(A_lat))-A_lat B_lat;D4 zeros(size(D4,1),size(B_lat,2))];
b4 = [zeros(size(A_lat,1),1);v4_tilde];

v4 = M4\b4; u4 = v4(5:6); v4 = v4(1:4);

% Feedback gain controller
K = real([u1,u2,u3,u4]/[v1,v2,v3,v4]);

% Verification
[V,D] = eig(A_lat-B_lat*K);

% Closed loop system
sysFull_lat_cl = feedback(sysFull_lat,K);


%% 3.5
Tsim_lat = 10;

% initial conditions
x01 = [1 0 0 0]';
x02 = [0 0 1 0]';

% open-loop simulation
[Y1,T1,X1] = initial(sysFull_lat,x01,Tsim_lat);
Y1 = rad2deg(Y1);
[Y2,T2,X2] = initial(sysFull_lat,x02,Tsim_lat);
Y2 = rad2deg(Y2);

% closed-loop simulation (feedback gain controller using eigenstructure
% assignment)
[Ycl1,Tcl1,Xcl1] = initial(sysFull_lat_cl,x01,Tsim_lat);
Ycl1 = rad2deg(Ycl1);
[Ycl2,Tcl2,Xcl2] = initial(sysFull_lat_cl,x02,Tsim_lat);
Ycl2 = rad2deg(Ycl2);

% plots
% initial condition x01
figure(7); clf;
label_y = ["$r [^\circ/s]$","$\beta [^\circ]$","$p [^\circ/s]$","$\Phi [^\circ]$"];
for i = 1:4
    subplot(4,1,i); grid on; hold all; ylabel(label_y(i),'Interpreter','latex','FontSize',12);
    plot(T1,Y1(:,i),'LineWidth',1.5,'Color',plot_colors(1,:),'LineStyle','-');
    plot(Tcl1,Ycl1(:,i),'LineWidth',1.5,'Color',plot_colors(1,:),'LineStyle','--');
end
hold off
xlabel('$t [s]$','Interpreter','latex','FontSize',12);
subplot(4,1,3); lgd = legend('open-loop','with feedback gain controller');
lgd.Location = 'northeast'; lgd.FontSize = 11;
lgd.Interpreter = 'latex'; lgd.NumColumns = 1;
subplot(4,1,1); title("Initial condition $x_{0,1} = [1,0,0,0]'$", ...
                      'Interpreter','latex', 'FontSize',14);

% initial condition x02
figure(8); clf;
label_y = ["$r [^\circ/s]$","$\beta [^\circ]$","$p [^\circ/s]$","$\Phi [^\circ]$"];
for i = 1:4
    subplot(4,1,i); grid on; hold all; ylabel(label_y(i),'Interpreter','latex','FontSize',12);
    plot(T2,Y2(:,i),'LineWidth',1.5,'Color',plot_colors(1,:),'LineStyle','-');
    plot(Tcl2,Ycl2(:,i),'LineWidth',1.5,'Color',plot_colors(1,:),'LineStyle','--');
end
hold off
xlabel('$t [s]$','Interpreter','latex','FontSize',12);
subplot(4,1,3); lgd = legend('open-loop','with feedback gain controller');
lgd.Location = 'northeast'; lgd.FontSize = 11;
lgd.Interpreter = 'latex'; lgd.NumColumns = 1;
subplot(4,1,1); title("Initial condition $x_{0,2} = [0,0,1,0]'$", ...
                      'Interpreter','latex', 'FontSize',14);
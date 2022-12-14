%% Task 3: Design of a base controller for decoupling lateral aircraft movement (SAS)

%% 3.2

Qc = ctrb(A_lat,B_lat);    % controllability matrix
rank(Qc);


%% 3.3

% original system
[Wn_Full_lat,Z_Full_lat,P_Full_lat] = damp(sysFull_lat);

% Roll time constant
eval_R = P_Full_lat(Wn_Full_lat==max(Wn_Full_lat));

% Dutch roll
%Z_DR = .3;
Z_DR = .7;
Wn_DR = Wn_Full_lat(2);
eval_DR = [-Z_DR*Wn_DR+1j*Wn_DR*sqrt(1-Z_DR^2) -Z_DR*Wn_DR-1j*Wn_DR*sqrt(1-Z_DR^2)];

% Spiral
T2_R_Full_lat = log(2)/P_Full_lat(P_Full_lat>0);
T2_R = T2_R_Full_lat*(1+.5); % increase spiral doubling time by 50%
eval_S = log(2)/T2_R;

desiredPoles = [eval_R, eval_DR, eval_S]; % Roll, Dutch roll, Spiral


%% 3.4

% Eigenstructure assignment: Roll
v_1d = [1 0 1 1]'; % [* 0 1 *]'
D1 = [0 1 0 0;0 0 1 0];
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
Tsim_lat = 100;

% initial conditions
x01 = [1 0 0 0]';
x02 = [0 0 1 0]';

% % open-loop simulation
% [Y1,T1,~] = initial(sysFull_lat,x01,Tsim_lat);
% Y1 = rad2deg(Y1);
% [Y2,T2,~] = initial(sysFull_lat,x02,Tsim_lat);
% Y2 = rad2deg(Y2);
% 
% % closed-loop simulation (feedback gain controller using eigenstructure
% % assignment)
% [Ycl1,Tcl1,~] = initial(sysFull_lat_cl,x01,Tsim_lat);
% Ycl1 = rad2deg(Ycl1);
% [Ycl2,Tcl2,~] = initial(sysFull_lat_cl,x02,Tsim_lat);
% Ycl2 = rad2deg(Ycl2);

plotResponseLateral(sysFull_lat_cl,x01,Tsim_lat);
plotResponseLateral(sysFull_lat_cl,x02,Tsim_lat);
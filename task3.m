%% Task 3: Design of a base controller for decoupling lateral aircraft movement (SAS)

%% 3.2

Qc = ctrb(A_lat,B_lat);    % controllability matrix
rank(Qc)


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
[V,D] = eig(A_lat-B_lat*K1)

% Closed loop system
sysFull_lat_ea = feedback(sysFull_lat,K)


%% 3.5


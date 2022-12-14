function plotResponseLateral(newSystem,x0,TFINAL,controlInput,inputDeflection,TSTART,TEND)
%plotResponseLateral  Simulate time response and plot the result
%
%   plotResponseLateral(newSystem,x0,TFINAL,controlInput,inputDeflection,TSTART,TEND)
%   simulates and plots the time response of the dynamic system newSystem
%   with initial condition x0 in rad or rad/s to the input signal described by
%   inputDeflection, TSTART, TEND, TFINAL.
% 
%   The controlInput is the order of control variable in the state-space form.
%   For example,
%
%       plotResponseLateral(sysFull_lat_cl,[0;0;1;0],50,1,5,5,10)
%
%   simulates the time response of the system sysFull_lat_cl
%   with initial condition of [0;0;1;0].
%
%   The controlInput of 1 means the aileron deflection as input,
%
%   inputDeflection is the deflection angle in ° of the input.
%   TSTART and TEND indicate the point in time when the input should start moving and
%   back to 0, respectively. TFINAL is the end simulation time.
%
%   All time variables are in second
%   
%   Other example,

%       plotResponseLateral(sysFull_lat_cl,[0,1,0,-1]',50)
%

global plot_colors

% orginal system
A_lat = [-.4956 1.9243 -.1206 0;
         -.9795 -.193 .0963 .1055;
         2.107 -5.5049 -3.3388 0;
         0 0 1 0];

B_lat = [-.4515 -1.318;
         0 .0362;
         -9.498 1.9929;
         0 0];

C = eye(4); D = 0;

sysFull_lat = ss(A_lat,B_lat,C,D);
sysFull_lat.StateName = {'r','\beta','p','\Phi'};
sysFull_lat.InputName = {'\delta_a','\delta_r'};
sysFull_lat.OutputName = sysFull_lat.StateName;
sysFull_lat.TimeUnit = 'seconds';

% simulation
if nargin == 3
    [Y_Full_lat,T_Full_lat,~] = initial(sysFull_lat,x0,TFINAL);
    [Y_newSystem,T_newSystem,~] = initial(newSystem,x0,TFINAL);

    plot_title = sprintf("Initial condition $x_{0} = [%.f,%.f,%.f,%.f]'$", ...
                         x0(1),x0(2),x0(3),x0(4));

else
    tsim = 0:.01:TFINAL;
    input = (tsim>=TSTART)*deg2rad(inputDeflection) + (tsim>=TEND)*deg2rad(-inputDeflection);

    [Y_Full_lat,T_Full_lat,~] = lsim(sysFull_lat(:,controlInput),input,tsim,x0);
    [Y_newSystem,T_newSystem] = lsim(newSystem(:,controlInput),input,tsim,x0);
end
Y_Full_lat = rad2deg(Y_Full_lat);
Y_newSystem = rad2deg(Y_newSystem);

% plot the result
h = gcf;
figure(h.Number+1); clf; 
label_y = ["$r [^\circ/s]$","$\beta [^\circ]$","$p [^\circ/s]$","$\Phi [^\circ]$"];
for i = 1:length(x0)
    subplot(length(x0),1,i); hold all; grid on; ylabel(label_y(i),'Interpreter','latex','FontSize',12);
    plot(T_Full_lat,Y_Full_lat(:,i),'LineWidth',1.5,'Color',plot_colors(1,:),'LineStyle','--');
    plot(T_newSystem,Y_newSystem(:,i),'LineWidth',1.5,'Color',plot_colors(1,:),'LineStyle','-');
end
hold off
xlabel('$t [s]$','Interpreter','latex','FontSize',12);
subplot(length(x0),1,length(x0)); lgd = legend('open-loop','with feedback gain controller');
lgd.Location = 'best'; lgd.FontSize = 11;
lgd.Interpreter = 'latex'; lgd.NumColumns = 2;
subplot(length(x0),1,1); title(plot_title,'Interpreter','latex','FontSize',14)
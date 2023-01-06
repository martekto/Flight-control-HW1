global plot_colors

folder = "C:\Users\marte\OneDrive\Documents\MATLAB\FlightControlWS22\FC-HW1";
filename = fullfile(folder,"sw_00370004_TUB614_0501.mat");
file_rp = load(filename);

time_rp = file_rp.data.wd_time;
data_rp = [file_rp.data.wd_rb; file_rp.data.wd_beta; file_rp.data.wd_pb; file_rp.data.wd_phi];

label_y = ["$r [^\circ/s]$","$\beta [^\circ]$","$p [^\circ/s]$","$\Phi [^\circ]$"];
ones_array = ones([1,26442])*0.5;
figure(10); clf
for i = 1:4
    subplot(4,1,i); hold all; grid on; 
    plot(time_rp,file_rp.data.wd_rb);
end
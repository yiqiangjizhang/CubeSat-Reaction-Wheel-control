clear;
close all;
clc;

set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaulttextinterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');

format short

load("data10004.mat");
input10004 = input10004{:,:};

num = 10004;

yaw_plot_coarse_fine = sprintf("plots/yaw_plot_coarse_fine%d.pdf", num);
gyro_z_plot = sprintf("plots/gyro_z_plot%d.pdf", num);

% Time
A(:,1) = input10004(:,1);
A(:,2) = input10004(:,2);
A(:,3) = input10004(:,3);

% Data
B = input10004(:,4:end);

gyroz = B(:,1);
yaw = B(:,2);


t = 3600 * A(:, 1) + 60 * A(:, 2) + A(:, 3);
t = t - t(1);

% PID output vs time
h = figure(1);
title("\textbf{Yaw angle}");
hold on;
plot(t, yaw, 'color', [0 167 83] / 255);
scatter(t, yaw, 10, [0 167 83] / 255, 'filled');
xlim([0 ceil(max(t))]);
xlabel("Time $\left( \mathrm{s} \right)$");
ylabel("Yaw angle $\left( \mathrm{deg} \right)$");
set(gcf, 'units', 'points', 'position', [100, 100, 560, 420]);

xline(0,'-',{'ACC Ramp start'});
xline(7.663,'-',{'Waiting'});
xline(10.679,'-',{'DEC Ramp Start'});
xline(25.304,'-',{'Positioning','Fine'});
yline(100,'-',{'Set point'});
box on;
grid on;
grid minor;
hold off;
box on;
hold off;

set(h, 'Units', 'Centimeters');
pos = get(h, 'Position');
set(h, 'PaperPositionMode', 'Auto', 'PaperUnits', 'Centimeters', ...
    'PaperSize', [pos(3), pos(4)]);
print(h, yaw_plot_coarse_fine, '-dpdf', '-r0');


% Angular velocity in z vs time
h = figure(2);
title("\textbf{Angular velocity $\omega_z$}");
hold on;
plot(t, gyroz, 'color', [0 167 83]/255);
scatter(t, gyroz, 10, [0 167 83]/255, 'filled');
xlim([0 ceil(max(t))]);
xlabel("Time $\left( \mathrm{s} \right)$");
ylabel("Angular velocity $\left( \mathrm{deg}/\mathrm{s} \right)$");
set(gcf,'units','points','position',[100,100,560,420]);

xline(0,'-',{'ACC Ramp start'});
xline(7.663,'-',{'Waiting'});
xline(10.679,'-',{'DEC Ramp Start'});
xline(25.304,'-',{'Positioning','Fine'});
box on;
grid on;
grid minor;
hold off;
box on;
hold off;

set(h, 'Units', 'Centimeters');
pos = get(h, 'Position');
set(h, 'PaperPositionMode', 'Auto', 'PaperUnits', 'Centimeters', ...
    'PaperSize',[pos(3), pos(4)]);
print(h, gyro_z_plot, '-dpdf', '-r0');
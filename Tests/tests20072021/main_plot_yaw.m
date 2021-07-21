clear;
close all;
clc;

set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaulttextinterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');

format short

num = 1009;
time_file = sprintf("time%d.txt", num);
data_file = sprintf("data%d.txt", num);
yaw_plot_coarse_fine = sprintf("plots/yaw_plot_coarse_fine%d.pdf", num);

A = importdata(time_file);
B = importdata(data_file);

pid = B(:, 1);

t = 3600 * A(:, 1) + 60 * A(:, 2) + A(:, 3);
t = t - t(1);

% PID putput vs time
h = figure(1);
title("\textbf{Yaw angle}");
hold on;
plot(t, pid, 'color', [0 167 83] / 255);
scatter(t, pid, 10, [0 167 83] / 255, 'filled');
xlim([0 ceil(max(t))]);
xlabel("Time $\left( \mathrm{s} \right)$");
ylabel("Yaw angle $\left( \mathrm{deg} \right)$");
set(gcf, 'units', 'points', 'position', [100, 100, 560, 420]);

xline(0,'-',{'ACC Ramp start'});
xline(7.0780,'-',{'Waiting'});
xline(10.7860,'-',{'DEC Ramp Start'});
xline(24.6940,'-',{'Positioning','Fine'});
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

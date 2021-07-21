clear;
close all;
clc;

set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaulttextinterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');

format short

num = 1008;
time_file = sprintf("time2%d.txt", num);
data_file = sprintf("pid_output%d.txt", num);
pid_output_plot = sprintf("plots/pid_output_plot%d.pdf", num);

A = importdata(time_file);
B = importdata(data_file);

yaw = B(:, 1);

t = 3600 * A(:, 1) + 60 * A(:, 2) + A(:, 3);
t = t - t(1);

% PID putput vs time
h = figure(1);
title("\textbf{PID output}");
hold on;
plot(t, yaw, 'color', [0 167 83] / 255);
scatter(t, yaw, 10, [0 167 83] / 255, 'filled');
xlim([0 ceil(max(t))]);
xlabel("Time $\left( \mathrm{s} \right)$");
ylabel("$|\mathrm{PID}$ $\mathrm{output}|$");
set(gcf, 'units', 'points', 'position', [100, 100, 560, 420]);

% xline(0,'-',{'Positioning','Fine'});
% yline(352,'-',{'Set point'});
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
print(h, pid_output_plot, '-dpdf', '-r0');

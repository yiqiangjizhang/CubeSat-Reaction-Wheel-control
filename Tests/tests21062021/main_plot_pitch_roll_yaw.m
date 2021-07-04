clear;
close all;
clc;

set(groot,'defaultAxesTickLabelInterpreter','latex');
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');

format short

num = 12;
time_file = sprintf("time%d.txt", num);
data_file = sprintf("data%d.txt", num);
yaw_plot = sprintf("plots/yaw%d.pdf", num);
pitch_plot = sprintf("plots/pitch%d.pdf", num);
roll_plot = sprintf("plots/roll%d.pdf", num);
all_plot = sprintf("plots/all%d.pdf", num);

A = importdata(time_file);
B = importdata(data_file);
pitch = B(:,1);
roll = B(:,2);
yaw = B(:,3);
t = 3600*A(:,1) + 60*A(:,2) + A(:,3);
t = t - t(1);

% Pitch angle vs time
h = figure(1);
title("\textbf{Pitch angle}");
hold on;
plot(t, pitch, 'b');
scatter(t, pitch, 10, 'b', 'filled');
xlim([0 ceil(max(t))]);
xlabel("Time $\left( \mathrm{s} \right)$");
ylabel("Pitch angle $\left( \mathrm{deg} \right)$");
set(gcf,'units','points','position',[100,100,560,420]);
xl1 = xline(0,'-',{'Ramp Start'});
xl1.LabelVerticalAlignment = 'middle';
xline(0.0830,'-',{'Waiting','Fine'});
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
print(h, pitch_plot, '-dpdf', '-r0');

% Roll angle vs time
h = figure(2);
title("\textbf{Roll angle}");
hold on;
plot(t, roll, 'r');
scatter(t, roll, 10, 'r', 'filled');
xlim([0 ceil(max(t))]);
xlabel("Time $\left( \mathrm{s} \right)$");
ylabel("Roll angle $\left( \mathrm{deg} \right)$");
set(gcf,'units','points','position',[100,100,560,420]);
xl1 = xline(0,'-',{'Ramp Start'});
xl1.LabelVerticalAlignment = 'middle';
xline(0.0830,'-',{'Waiting','Fine'});
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
print(h, roll_plot, '-dpdf', '-r0');

% Yaw angle vs time
h = figure(3);
title("\textbf{Yaw angle}");
hold on;
plot(t, yaw, 'color', [0 167 83]/255);
scatter(t, yaw, 10, [0 167 83]/255, 'filled');
xlim([0 ceil(max(t))]);
xlabel("Time $\left( \mathrm{s} \right)$");
ylabel("Yaw angle $\left( \mathrm{deg} \right)$");
set(gcf,'units','points','position',[100,100,560,420]);
xl1 = xline(0,'-',{'Ramp Start'});
xl1.LabelVerticalAlignment = 'middle';
xline(0.0830,'-',{'Waiting','Fine'});
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
print(h, yaw_plot, '-dpdf', '-r0');

% All angles vs time
h = figure(4);
title("\textbf{Attitude -- Orientation}");
hold on;
plot(t, pitch, 'b');
plot(t, roll, 'r');
plot(t, yaw, 'color', [0 167 83]/255);
scatter(t, pitch, 10, 'b', 'filled');
scatter(t, roll, 10, 'r', 'filled');
scatter(t, yaw, 10, [0 167 83]/255, 'filled');
xlim([0 ceil(max(t))]);
xlabel("Time $\left( \mathrm{s} \right)$");
ylabel("Angle $\left( \mathrm{deg} \right)$");
set(gcf,'units','points','position',[100,100,560,420]);
xl1 = xline(0,'-',{'Ramp Start'});
xl1.LabelVerticalAlignment = 'middle';
xline(0.0830,'-',{'Waiting','Fine'});
box on;
grid on;
grid minor;
hold off;
box on;
legend("Pitch angle", "Roll angle", "Yaw angle");
hold off;

set(h, 'Units', 'Centimeters');
pos = get(h, 'Position');
set(h, 'PaperPositionMode', 'Auto', 'PaperUnits', 'Centimeters', ...
    'PaperSize',[pos(3), pos(4)]);
print(h, all_plot, '-dpdf', '-r0');
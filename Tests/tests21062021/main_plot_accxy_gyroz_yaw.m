clear;
close all;
clc;

set(groot,'defaultAxesTickLabelInterpreter','latex');
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');

format short

num = 16;
time_file = sprintf("time%d.txt", num);
data_file = sprintf("data%d.txt", num);
yaw_plot = sprintf("plots/yaw%d.pdf", num);
acc_xy_plot = sprintf("plots/acc_xy%d.pdf", num);
gyro_z_plot = sprintf("plots/gyro_z%d.pdf", num);


A = importdata(time_file);
B = importdata(data_file);

accx = B(:,1);
accy = B(:,2);
gyroz = B(:,3);
yaw = B(:,4);

t = 3600*A(:,1) + 60*A(:,2) + A(:,3);
t = t - t(1);


% Acceleration in x annd y vs time
h = figure(1);
title("\textbf{Acceleration in $x$ and $y$ direction}");
hold on;
plot(t, accx, 'b');
plot(t, accy, 'r');
scatter(t, accx, 10, 'b', 'filled');
scatter(t, accy, 10, 'r', 'filled');
xlabel("Time $\left( \mathrm{s} \right)$");
ylabel("Acceleration $\left( \mathrm{m}/\mathrm{s} \right)$");
set(gcf,'units','points','position',[100,100,1200,420]);

xline(0,'-',{'Positioning','Fine'});
xline(352,'-',{'Set point'});
box on;
grid on;
grid minor;
hold off;
box on;
legend("Acceleration in $x$", "Acceleration in $y$");
hold off;

set(h, 'Units', 'Centimeters');
pos = get(h, 'Position');
set(h, 'PaperPositionMode', 'Auto', 'PaperUnits', 'Centimeters', ...
    'PaperSize',[pos(3), pos(4)]);
print(h, acc_xy_plot, '-dpdf', '-r0');



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

xline(0,'-',{'ACC Ramp start / Waiting'});
xline(3.31,'-',{'DEC Ramp Start'});
xline(12.299,'-',{'Positioning','Fine'});
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

xline(0,'-',{'Positioning','Fine'});
yline(352,'-',{'Set point'});
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

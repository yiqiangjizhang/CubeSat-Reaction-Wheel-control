clear;
close all;
clc;

data = importdata('data_01.txt');
data(1,:) = [];
t = data(:,1);
accx = data(:,2);
accy = data(:,3);
degz = data(:,4);
gyrz = data(:,5);

set(groot,'defaultAxesTickLabelInterpreter','latex');
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');

% Acceleration in X and Y axis
h = figure(1);
title("\textbf{Acceleration in $X$ and $Y$ axis}");
hold on;
plot(t, accx, 'b');
plot(t, accy, 'r');
scatter(t, accx, 10, 'b', 'filled');
scatter(t, accy, 10, 'r', 'filled');
xlim([8 17]);
xlabel("Time $\left( \mathrm{s} \right)$");
ylabel("Acceleration $\left( \mathrm{m} / \mathrm{s}^2 \right)$");
set(gcf,'units','points','position',[100,100,560,420]);

xline(8.74,'-',{'ACC Ramp Start'});
xline(10.89,'-',{'Waiting'});
xline(15.45,'-',{'DECC Ramp Start'});
box on;
grid on;
grid minor;
hold off;
box on;
legend("Acceleration in $X$ axis", "Acceleration in $Y$ axis", 'Location', "Southwest");
hold off;

set(h, 'Units', 'Centimeters');
pos = get(h, 'Position');
set(h, 'PaperPositionMode', 'Auto', 'PaperUnits', 'Centimeters', ...
    'PaperSize',[pos(3), pos(4)]);
print(h, "acceleration_x_and_y_axis.pdf", '-dpdf', '-r0');

% Degree Z
h = figure(2);
title("\textbf{Yaw measure}");
hold on;
plot(t, degz, 'color', [0 167 83]/255);
scatter(t, degz, 10, [0 167 83]/255, 'filled');
xlim([8 17]);
xlabel("Time $\left( \mathrm{s} \right)$");
ylabel("Yaw angle $\left( \mathrm{deg} \right)$");
set(gcf,'units','points','position',[100,100,560,420]);

xline(8.74,'-',{'ACC Ramp Start'});
xline(10.89,'-',{'Waiting'});
xline(15.45,'-',{'DECC Ramp Start'});
box on;
grid on;
grid minor;
hold off;
box on;
% legend("Acceleration in $X$ axis", "Acceleration in $Y$ axis", 'Location', "Northwest");
hold off;

set(h, 'Units', 'Centimeters');
pos = get(h, 'Position');
set(h, 'PaperPositionMode', 'Auto', 'PaperUnits', 'Centimeters', ...
    'PaperSize',[pos(3), pos(4)]);
print(h, "yaw_measure.pdf", '-dpdf', '-r0');

% Gyro Z
h = figure(3);
title("\textbf{Angular speed in Z axis}");
hold on;
plot(t, gyrz, 'color', [0 167 83]/255);
scatter(t, gyrz, 10, [0 167 83]/255, 'filled');
xlim([8 17]);
xlabel("Time $\left( \mathrm{s} \right)$");
ylabel("Angular speed $\left( \mathrm{deg} / \mathrm{s} \right)$");
set(gcf,'units','points','position',[100,100,560,420]);

xline(8.74,'-',{'ACC Ramp Start'});
xline(10.89,'-',{'Waiting'});
xline(15.45,'-',{'DECC Ramp Start'});
box on;
grid on;
grid minor;
hold off;
box on;
% legend("Acceleration in $X$ axis", "Acceleration in $Y$ axis", 'Location', "Northwest");
hold off;

set(h, 'Units', 'Centimeters');
pos = get(h, 'Position');
set(h, 'PaperPositionMode', 'Auto', 'PaperUnits', 'Centimeters', ...
    'PaperSize',[pos(3), pos(4)]);
print(h, "gyro_z.pdf", '-dpdf', '-r0');
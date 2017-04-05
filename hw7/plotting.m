clear; clc; close all;

[x, y] = textread('tau.txt', '%f %f', 17);

% least square estimation
p = polyfit(x,y,1);
frame = 0:50;
tau = polyval(p,frame);

figure(1); hold on; grid on;
title('Time to Impact(in frames)');
scatter(x,y);
plot(frame,tau);
legend('measured', 'estimated');
xlabel('Frame Number');
ylabel('Estimated Frames to Impact');
xlim([0 50]);
ylim([-10 50]);

impact_frame = -p(2)/p(1)

%% task2
distance = 15.25;   % 15.25mm

tau = tau*distance;
figure(2); hold on; grid on;
title('Time to Impact(in distance)');
scatter(x,y*distance);
plot(frame,tau);
legend('measured', 'estimated');
xlabel('Frame Number');
ylabel('Estimated Distance to Impact(mm)');
xlim([0 50]);
ylim([-10 700]);

%% task3
[frame, distance] = textread('distance.txt', '%f %f', 18);
% least square estimation
p = polyfit(frame,distance,1);
tau = polyval(p,frame);

figure(3); hold on; grid on;
title('Distance(mm)');
scatter(frame,distance);
plot(frame,tau);
legend('measured', 'estimated');
xlabel('Frame Number');
ylabel('Distance(mm)');
xlim([1 18]);
ylim([300 700]);

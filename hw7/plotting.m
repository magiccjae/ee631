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
distance = 1.525;   % 15.25mm = 1.525cm

tau = tau*distance;
figure(2); hold on; grid on;
title('Time to Impact(in distance)');
scatter(x,y*distance);
plot(frame,tau);
legend('measured', 'estimated');
xlabel('Frame Number');
ylabel('Estimated Distance to Impact(in centimeter)');
xlim([0 50]);
ylim([-10 70]);

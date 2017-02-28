clear; clc; close all;

[x, y, z] = textread('xyz.txt', '%f %f %f', 28);
y = -y;

% least square estimation
p = polyfit(z,x,1);
z2 = 0:0.1:450;
x2 = polyval(p,z2);

figure(1); hold on;
title('x vs z');
scatter(z,x);
plot(z2,x2);
legend('measured', 'estimated');
xlabel('z(inch)');
ylabel('x(inch)');
xlim([0 450]);
ylim([0 1]);

p2 = polyfit(z,y,2);
y2 = polyval(p2,z2);

figure(2); hold on;
title('y vs z');
scatter(z,y);
plot(z2,y2);
legend('measured', 'estimated');
xlabel('z(inch)');
ylabel('y(inch)');
xlim([0 450]);
ylim([0 80]);

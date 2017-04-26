clear; close all; clc;

figure(1); hold on;
title('visual odometry');
xlim([-300 300]);
ylim([-300 300]);

vo = load('/home/magiccjae/jae_stuff/classes/ee631/hw8/build/VO_result2.txt');
Tx_vo = vo(:,4);
Tz_vo = vo(:,12);
plot(Tx_vo, Tz_vo);
xlabel('X');
ylabel('Z');

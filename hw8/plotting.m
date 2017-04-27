clear; close all; clc;

truth = load('/home/magiccjae/jae_stuff/classes/ee631/hw8/VO Practice Sequence R and T.txt');

Tx = truth(:,4);
Tz = truth(:,12);

figure(1); hold on;
title('visual odometry');
xlim([-300 300]);
plot(Tx,Tz);

vo = load('/home/magiccjae/jae_stuff/classes/ee631/hw8/VO_task1.txt');
Tx_vo = vo(:,4);
Tz_vo = vo(:,12);
plot(Tx_vo, Tz_vo);
xlabel('X');
ylabel('Z');
legend('truth', 'VO');
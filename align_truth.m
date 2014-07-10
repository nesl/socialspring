%%{
%% Housekeeping
clc; close all; clear all;

%% Add paths
addpath('classes');

%% Read data
[accel, grav, gyro, accel_lin, mag, euler,...
    gps, displacement, speed, heading] = parseRawData('data/rawdata');

%%}
%% Estimated displacement vector
[estimate] = getEstimatedDisplacement(accel_lin, gyro);

%%
OFFSET = 20;

cfigure(30,15);

subplot(1,2,1);

h_est = plot([0], [0], 'o-b');
xlim([-15 15]);
ylim([-15 15]);

subplot(1,2,2);

h_true = plot([0], [0], 'o-r');
xlim([-5 25]);
ylim([-5 25]);

for i=1:size(estimate,1);
    xy_est = estimate(i,2:3);
    t = estimate(i,1) + OFFSET;
    xy_true = getRealCoordinate('../Ground truth annotation/output/final_01.mat', t);

    subplot(1,2,1);
    hold on;
    plot(xy_est(1), xy_est(2), 'o-b');
    
    subplot(1,2,2);
    hold on;
    plot(xy_true(1), xy_true(2), 'o-r');
    pause(0.01);
end

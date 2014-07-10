rng(3);
%{
%% Housekeeping
clc; close all; clear all;

%% Add paths
addpath('classes');

%% Read data
[accel, grav, gyro, accel_lin, mag, euler,...
    gps, displacement, speed, heading] = parseRawData('data/rawdata');

%}
%% Estimated displacement vector
[estimate] = getEstimatedDisplacement(accel_lin, gyro);

%% Animate
%{
cfigure(20,20);
h = plot([0],[0], 'o-b','MarkerFaceColor','b');
grid on;
xlim([-15 10]);
ylim([-5 15]);
xlabel('X Position','FonTSize',14);
ylabel('Y Position','FontSize',14);

wlen = 20;

for i=1:size(estimate,1)
    start = max(1, i-wlen);
    stop = i;
    
    x = estimate(start:stop,2);
    y = estimate(start:stop,3);
    
    set(h, 'XData', x, 'YData', y);
    pause(0.05);
end

%}

%% Sample random segments
%rng(1);
ave_len = 10; % steps
num_paths = 5;
% IMPORTANT OFFSET TIME TO ALIGN GROUND TRUTH AND SAMPLES
t_offset = 0;

path_starts = randi( size(estimate,1) - ave_len, num_paths, 1);
path_lengths = round( 20 + 10*randn(num_paths, 1) );
path_ends = min( size(estimate,1), path_starts + path_lengths );


paths = {}; % x,y
for i=1:num_paths
    start = path_starts(i);
    stop = path_ends(i);
    
    xy = estimate(start:stop, 2:3);
    
    paths{i} = xy;
end

%% get true coordinates
true = {}; % x,y
OFFSET = 20;
for i=1:num_paths
    start = path_starts(i);
    
    xy = [];
    for j=1:path_lengths(i)
        t = estimate(start + j - 1, 1) + OFFSET;
        gt = getRealCoordinate('../Ground truth annotation/output/final_01.mat', t);
        xy = [xy; gt];
    end
    
    true{i} = xy;
end

% correct so the samples start with zero error.
for i=1:num_paths
    start_true = true{i}(1,:);
    start_est = paths{i}(1,:);
    
    correction = start_est - start_true;
    
    for j=1:path_lengths(i)
        true{i}(j,:) = true{i}(j,:) + correction;
    end
    
end


% plot random segments
cfigure(20,20);
h = plot([0],[0], 'o-b','MarkerFaceColor','b');
grid on;
xlim([-15 10]);
ylim([-5 15]);
xlabel('X Position','FonTSize',14);
ylabel('Y Position','FontSize',14);
colors = hsv(num_paths);

%%{
for i=1:num_paths
    xy = paths{i};
    plot(xy(:,1), xy(:,2), 'o-', 'Color', colors(i,:), 'LineWidth',2);
    hold on;
    tru = true{i};
    plot(tru(:,1), tru(:,2), 'o--', 'Color', 0.7*colors(i,:), 'LineWidth',2);
end
%%}

%% Create path objectss

%% Encounters
e_radius = 3;

for i=1:num_paths
    for j=1:num_paths
        
        if i == j
            continue;
        end
        
        
    end
end






% TODO...


% get true "encounter distances"
% TODO...

% add noise to encounter distances
% TODO...

% constrain using springs
% TODO...

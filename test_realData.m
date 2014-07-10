rng(5);
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
path_len = 15; % steps
num_paths = 5;

path_starts = randi( size(estimate,1) - path_len, num_paths, 1);
path_ends = min( size(estimate,1), path_starts + path_len );


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
    for j=1:path_len
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
    
    for j=1:path_len
        true{i}(j,:) = true{i}(j,:) + correction;
    end
    
end


% plot random segments
cfigure(10,10);
h = plot([0],[0], 'o-b','MarkerFaceColor','b');

colors = hsv(num_paths);

%%{
scale = 0.5;
for i=1:num_paths
    xy = paths{i};
    plot(scale*(xy(:,1) + 10), scale*(xy(:,2) + 10), 'o-', 'Color', colors(i,:), 'LineWidth',2);
    hold on;
    tru = true{i};
    plot(scale*(tru(:,1) + 10), scale*(tru(:,2) + 10), 's:', 'Color', 0.7*colors(i,:), 'LineWidth',2);
end
grid on;
xlim([1 8]);
ylim([0 11]);
xlabel('X Position (m)','FonTSize',14);
ylabel('Y Position (m)','FontSize',14);
legend('Path 1 est', 'Path 1 true');
%%}
%saveplot('figures/real_samples');


%% Create path objects
spring_paths = {};
POS_OFFSET = [10 10];
GRID_X = 25;
GRID_Y = 25;

for n=1:num_paths
    spring_paths{n} = Path();
    
    xy = paths{n};
    for i=1:size(xy,1)
        node = PathNode(xy(i,:) + POS_OFFSET, GRID_X, GRID_Y);
        
        stiffness = 10;
        if i == 1
            node.setAbsoluteCertainty(stiffness);
        else
            node.setAbsoluteCertainty(stiffness/i);
        end
        
        node.setDamping(30);
        node.setOutgoingCertainty(1);
        node.setIncomingCertainty(1);
        
        spring_paths{n}.addNodeToEnd(node);
    end
    
end

%% Encounters
e_radius = 5;

for i=1:path_len
    
    % check if any nodes are close enough
    for n1=1:num_paths
        for n2=1:num_paths
            
            if n1==n2
                continue;
            end
            
            x1 = true{n1}(i,:);
            x2 = true{n2}(i,:);
            dist = norm(x1-x2);
            
            if dist <= e_radius
                % encounter!
                spring_paths{n1}.nodes(i).constrainToNode(...
                    spring_paths{n2}.nodes(i), dist, 10);
            end
        end
    end
    
    
end


%% Let the solver do its thang
FRAMERATE = 120;
dt = (1/FRAMERATE);
t_end = 0.5; % seconds


% Set up plot
fig = cfigure(15,15);
set(gcf,'color','w');
hold on;
colors = hsv(num_paths);

path_handles = zeros(num_paths,1);

for i=1:num_paths
    path_handles(i) = plot([0], [0], 'o-', 'MarkerSize', 10, 'Color', colors(i,:), 'MarkerFaceColor', colors(i,:));
end

grid on;
xlim([0 GRID_X]);
ylim([0 GRID_Y]);
xlabel('X Position','FontSize',14);
ylabel('Y Position','FontSize',14);
h_title = title('Time = 0 sec','FontSize',15);

for t=0:dt:t_end
    
    % pause for plotting
    pause(.01);
    
    % dynamics
    for i=1:num_paths
        spring_paths{i}.moveNodesAccordingToForce(dt);
    end
    
    % update node plot locations
    for i=1:num_paths
        [x,y] = spring_paths{i}.getNodeCoordinates();
        set(path_handles(i), 'XData', x, 'YData', y);
    end

    % update title
    title_str = sprintf('Time = %.2f sec', t);
    set(h_title,'String', title_str);
    
    % refresh plot
    drawnow;
    
end


%% Calculate statistics

errors_before = [];
errors_after = [];

for agent_id=1:num_paths
    est_coords = paths{agent_id};
    tru_coords = true{agent_id};
    [x,y] = spring_paths{agent_id}.getNodeCoordinates();
    fix_coords = [x' y'];
    fix_coords(:,1) = fix_coords(:,1) - POS_OFFSET(1);
    fix_coords(:,2) = fix_coords(:,2) - POS_OFFSET(2);
    
    for i=1:size(tru_coords,1)
        e_before = norm(est_coords(i,:) - tru_coords(i,:));
        e_after = norm(fix_coords(i,:) - tru_coords(i,:));
        
        errors_before = [errors_before; e_before];
        errors_after = [errors_after; e_after];
    end
    
end


cfigure(10,10);
plot(scale*sort(errors_before,'descend'), 'o-b');
hold on;
plot(scale*sort(errors_after,'descend'), 's-r');
grid on;
xlabel('Sorted node ID','FontSize',14);
ylabel('Error (m)','FontSize',14);
legend('Before Refinement','After Refinement','Location','NE');
%xlim([0 30]);
%saveplot('figures/error_correction_REAL_5m_improve');

fprintf('from E = %.2f to E = %.2f\n (%.2f %%)\n',...
    mean(errors_before), mean(errors_after),...
    100*( mean(errors_after) - mean(errors_before) )/mean(errors_before));











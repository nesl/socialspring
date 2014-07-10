%% Housekeeping
clc; close all; clear all;

%% Add paths
addpath('classes');

%% Set up environment
GRID_X = 10;
GRID_Y = 10;
FRAMERATE = 30;
dt = (1/FRAMERATE);
t_end = 5; % seconds
SAVE_MOVIE = false;

%% Create test paths

p1_xy = [
    2 2
    4 4
    6 6
    ];


% create path 1
p1 = Path();
for i=1:size(p1_xy,1)
    node = PathNode(p1_xy(i,:), GRID_X, GRID_Y);
    if i == 1 || i == size(p2_xy,1)
        node.setAbsoluteCertainty(10);
    else
        node.setAbsoluteCertainty(1);
    end
    
    node.setOutgoingCertainty(1);
    node.setIncomingCertainty(1);
    node.setAngleCertainty(1.5);
    
    p1.addNodeToEnd(node);
    
end

% create path 2
p2 = Path();
for i=1:size(p2_xy,1)
    node = PathNode(p2_xy(i,:), GRID_X, GRID_Y);
    if i == 1 || i == size(p2_xy,1)
        node.setAbsoluteCertainty(10);
    else
        node.setAbsoluteCertainty(1);
    end
    
    node.setOutgoingCertainty(1);
    node.setIncomingCertainty(1);
    node.setAngleCertainty(1.5);
    
    p2.addNodeToEnd(node);
end

%% Set up movie
if SAVE_MOVIE
    vidObj = VideoWriter('outputs/angleconstrainhard.avi');
    vidObj.FrameRate=FRAMERATE;
    open(vidObj);
end


%% Set up plot
fig = cfigure(20,20);
set(gcf,'color','w');
hold on;
h_p1_nodes = plot([0], [0], 'o-b', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
h_p2_nodes = plot([0], [0], 's-r', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
grid on;
xlim([0 GRID_X]);
ylim([0 GRID_Y]);
xlabel('X Position','FontSize',14);
ylabel('Y Position','FontSize',14);
h_title = title('Time = 0 sec','FontSize',15);

%% Run simulation
oneshot = 1;
oneshot2 = 1;

% ============ START SIMULATION ============
for t=0:dt:t_end

    % pause for plotting
    pause(.01);
    
    % move a node
    if t > 1 && oneshot == 1
        oneshot = 0;
        %p2.nodes(end).xy = [8.5 7];
        %p1.nodes(2).xy = [2 1];
        
        p1.nodes(3).constrainToNode(p2.nodes(3), 0, 15);
    end
    
    if t > 3 && oneshot2 == 1
        oneshot2 = 0;
        p1.nodes(5).constrainToNode(p2.nodes(5), 0, 15);
    end
        
    % dynamics
    p1.moveNodesAccordingToForce(dt);
    p2.moveNodesAccordingToForce(dt);
    
    % update node locations
    [x1,y1] = p1.getNodeCoordinates();
    [x2,y2] = p2.getNodeCoordinates();
    set(h_p1_nodes, 'XData', x1, 'YData', y1);
    set(h_p2_nodes, 'XData', x2, 'YData', y2);
    
    % update title
    title_str = sprintf('Time = %.2f sec', t);
    set(h_title,'String', title_str);
    
    % refresh plot
    drawnow;
    
    % append to movie
    if SAVE_MOVIE
        f = getframe(fig);
        writeVideo(vidObj,f);
    end
    
end

if SAVE_MOVIE
    close(vidObj);
end














































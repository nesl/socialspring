%% Housekeeping
clc; close all; clear all;

%% Add paths
addpath('classes');

%% Set up environment
GRID_X = 10;
GRID_Y = 10;
FRAMERATE = 120;
dt = (1/FRAMERATE);
t_end = 50; % seconds
SAVE_MOVIE = false;

%% Create test paths

p1_xy = [
    1 2
    5 7
    8 1
    ];


% create path 1
p1 = Path();
for i=1:size(p1_xy,1)
    node = PathNode(p1_xy(i,:), GRID_X, GRID_Y);
    node.setDamping(10);
    node.setAbsoluteCertainty(0);

    node.setOutgoingCertainty(1);
    node.setIncomingCertainty(1);
    node.setAngleCertainty(0);
    
    p1.addNodeToEnd(node);
    
end

p1.nodes(end).linkOutgoingNode(p1.nodes(1));
p1.nodes(1).linkIncomingNode(p1.nodes(end));


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
        
        p1.nodes(1).xy = [1 7];
    end
    
        
    % dynamics
    p1.moveNodesAccordingToForce(dt);
    
    % update node locations
    [x1,y1] = p1.getNodeCoordinates();
    set(h_p1_nodes, 'XData', x1, 'YData', y1);
    
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














































%% Housekeeping
clc; close all; clear all;

%% Add paths
addpath('classes');

%% Set up environment
GRID_X = 10;
GRID_Y = 10;
FRAMERATE = 30;
dt = 1/150;
t_end = 2; % seconds
SAVE_MOVIE = true;

%% Create Map
m = zeros(GRID_X,GRID_Y);
m(3:10,5) = 1; %x,y

map = BinaryMap(GRID_X, GRID_Y);
map.setMap(m);

%% Create test paths

p1_xy = [
    1 5
    2.5 5.5
    4 6
    5.5 6.5
    7 7
    8.5 7.5
    ];


p2_xy = [
    2 1
    3.5 1.5
    5 2
    6.5 2.5
    8 3
    9.5 3.5
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
    node.setDamping(30);
    
    node.setOutgoingCertainty(1);
    node.setIncomingCertainty(1);
    node.setAngleCertainty(0.75);
    
    % set map
    node.setMap(map);
    
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
    node.setDamping(30);

    node.setOutgoingCertainty(1);
    node.setIncomingCertainty(1);
    node.setAngleCertainty(0.75);
    
    % set map
    node.setMap(map);
    
    p2.addNodeToEnd(node);
end

%% Set up movie
if SAVE_MOVIE
    vidObj = VideoWriter('outputs/wall.avi');
    vidObj.FrameRate=FRAMERATE;
    open(vidObj);
end


%% Set up plot
fig = cfigure(20,20);
set(gcf,'color','w');
hold on;
% map
[xw,yw] = map.getWallCoords();
plot(xw,yw,'sk','MarkerSize',55,'MarkerFaceColor',[0.6 0.6 0.6]);
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
    if t > 0.5 && oneshot == 1
        oneshot = 0;
        %p2.nodes(end).xy = [8.5 7];
        %p1.nodes(2).xy = [2 1];
        
        p1.nodes(3).constrainToNode(p2.nodes(3), 0, 15);
    end
    
    if t > 1 && oneshot2 == 1
        oneshot2 = 0;
        p1.nodes(4).constrainToNode(p2.nodes(4), 0, 15);
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














































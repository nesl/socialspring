%% Housekeeping
clc; close all; clear all;

%% Add paths
addpath('classes');

%% Vector Map
vm = VectorMap();

%% Create polygons

% table 
xv = [2 1 1 2 8 9 9 8];
yv = [8 7 5 4 4 5 7 8];
vm.addPolygon(xv,yv);

% trash
xv = [6 6 7.5 7.5];
yv = [1 0 0 1];
vm.addPolygon(xv,yv);

% recycling
xv = [8 8 9.5 9.5];
yv = [1 0 0 1];
vm.addPolygon(xv,yv);

% wall jut-out south
xv = [10 10 12 12];
yv = [1.5 0 0 1.5];
vm.addPolygon(xv,yv);

% temp chamber
xv = [12 12 14 14];
yv = [2.5 0 0 2.5];
vm.addPolygon(xv,yv);

% work bench
xv = [14.5 14.5 21 21];
yv = [3 0 0 3];
vm.addPolygon(xv,yv);

% storage
xv = [21 21 23 23];
yv = [2 0 0 2];
vm.addPolygon(xv,yv);

% soldering bench
xv = [23 23 29 29];
yv = [3 0 0 3];
vm.addPolygon(xv,yv);

% water cooler
xv = [4 4 5 5];
yv = [13 12 12 13];
vm.addPolygon(xv,yv);

% bookshelf, central desks, and couch
xv = [5 5 8 8 14 14 24 24 21 20 20 16 16 15];
yv = [16 12 12 10 10 10.5 10.5 16 16 17 20 20 17 16];
vm.addPolygon(xv,yv);

% bookshelf on left
xv = [0 0 1 1];
yv = [18 16 16 18];
vm.addPolygon(xv,yv);

% printer
xv = [0 0 2 2];
yv = [23 21.5 21.5 23];
vm.addPolygon(xv,yv);

% water tank
xv = [0 0 3.5 3.5];
yv = [30.5 27 27 30.5];
vm.addPolygon(xv,yv);

% peripheral desks
xv = [10 10 3 3 12 12 15 16 16 20 20 21 24 24 25 25 27 27 31 32 32 31 26 26 31 31 32.5 32.5 35 35];
yv = [30.5 24.5 24.5 22 22 27 27 26 23 23 26 27 27 28 28 26.5 26.5 27 27 26 19 18 18 16 16 5 5 1.5 1.5 30.5]; 
vm.addPolygon(xv,yv);

% borders
% L
xv = [-1 -1 0 0];
yv = [30.5 0 0 30.5];
vm.addPolygon(xv,yv);
% B
xv = [0 0 35 35];
yv = [0 -1 -1 0];
vm.addPolygon(xv,yv);
% R
xv = [35 35 36 36];
yv = [30.5 0 0 30.5];
vm.addPolygon(xv,yv);
% T
xv = [0 0 36 36];
yv = [31 30.5 30.5 31];
vm.addPolygon(xv,yv);



%% Plot vector map

m = vm.rasterize(35*5,30*5);



%% Get real paths
%start = [0 0];
true = [];
OFFSET = 20;
close all;

cfigure(17,17);

imshow(~m);
axis equal;

%%
start = [0 0 ];

for t = OFFSET + 0:0.25:339
    
    gt = getRealCoordinate('../Ground truth annotation/output/final_01.mat', t);
    true = [true; start + gt];
    
end

hold on;
scale = 10;
tru = fliplr(true);
plot(scale*tru(:,2), 173 - scale*tru(:,1), 'o-r', 'MarkerFaceColor', [1 0.5 0.5], 'MarkerSize',5);
axis normal;
xlabel('X Position','FontSize',14);
ylabel('Y Position','FontSize',14);

%saveplot('figures/true_paths');

classdef BinaryMap < handle
    %BINARYMAP Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (GetAccess = private)
        GRID_X
        GRID_Y
        map
    end
    
    methods
        % constructor
        function obj = BinaryMap(GRID_X, GRID_Y)
            obj.GRID_X = GRID_X;
            obj.GRID_Y = GRID_Y;
            obj.map = zeros(GRID_X, GRID_Y);
        end
        
        % modifications
        function setMap(obj, newmap)
            if size(newmap) ~= [obj.GRID_X obj.GRID_Y]
                error('specified map has improper dimensions');
            else
                obj.map = newmap;
            end
        end
        
        function addObstruction(obj, x, y)
            % make sure it's within bounds
            if x > 0 && y > 0 && x < obj.GRID_X && y < obj.GRID_Y
                obj.map( round(x), round(y) ) = 1;
            end
        end
        
        function removeObstruction(obj, x, y)
            % make sure it's within bounds
            if x > 0 && y > 0 && x < obj.GRID_X && y < obj.GRID_Y
                obj.map( round(x), round(y) ) = 0;
            end
        end
        
        % accessors
        function [res] = isBlocked(obj, xy)
            % first round the coordinates
            xr = round( xy(1) );
            yr = round( xy(2) );
            
            % if it's out of bounds, it's not blocked
            if xr <= 0 || yr <= 0 || xr > obj.GRID_X || yr > obj.GRID_Y
                res = 0;
            else
                % otherwise check the map
                res = obj.map(xr, yr);
            end
        end
        
        function [xy_new] = traverseMap(obj, xy, velocity)
            % we need to reshape the motion path of an object, taking the
            % binary map into account.
            
            % decompose velocity vector
            vel_x = [velocity(1) 0];
            vel_y = [0 velocity(2)];
            
            % Travel as far as we can in both x and y directions
            if ~obj.isBlocked(xy + vel_x)
                xy = xy + vel_x;
            end
            
            if ~obj.isBlocked(xy + vel_y)
                xy = xy + vel_y;
            end
            
            xy_new = xy;
            
        end
        
        function [free] = isPathFree(obj, start, stop)
            % choose an adequately small step size
            num_points = max(obj.GRID_X, obj.GRID_Y)*2;
            xline = linspace(start(1), stop(1), num_points);
            yline = linspace(start(2), stop(2), num_points);
            
            free = true;
            for i=1:num_points
                point = [xline(i) yline(i)];
                if obj.isBlocked(point)
                    free = false;
                    break;
                end
            end
        end
        
        function [x,y] = getWallCoords(obj)
            [x,y] = find(obj.map);
        end
        
        
    end
    
end

























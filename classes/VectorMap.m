classdef VectorMap < handle
    %VECTORMAP Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % cell array of polygon matrices
        polygons = {};
        max_x;
        max_y;
    end
    
    methods
        function obj = VectorMap()
            % init variables
            obj.max_x = 0;
            obj.max_y = 0;
        end
        
        % add a blocked polygon
        function addPolygon(obj, xv, yv)
            % update max x and y
            obj.max_x = max( obj.max_x, max(xv) );
            obj.max_y = max( obj.max_y, max(yv) );
            
            % add polygon
            obj.polygons = [obj.polygons;
                [xv;yv]];
        end
        
        % check to see if a coord is blocked
        function [res] = isBlocked(obj, x, y)
            res = 0;
            for i=1:length(obj.polygons)
                xv = obj.polygons{i}(1,:);
                yv = obj.polygons{i}(2,:);
                
                if inpolygon(x,y,xv,yv)
                    res = 1;
                    return;
                end
                
            end
        end
        
        % rasterize the map
        function [map] = rasterize(obj, NUM_X, NUM_Y)
            map = zeros(NUM_X, NUM_Y);
            % create linear discretization vectors
            xvec = linspace(0,obj.max_x, NUM_X);
            yvec = linspace(0,obj.max_y, NUM_Y);
            
            for xidx=1:NUM_X
                x = xvec(xidx);
                for yidx=1:NUM_Y
                    y = yvec(yidx);
                    map(xidx,yidx) = obj.isBlocked(x,y);
                end
            end
            
        end
        
    end
    
end


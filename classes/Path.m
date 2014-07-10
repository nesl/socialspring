classdef Path < handle
    %PATH to be used with PathNode objects
    
    properties (GetAccess = public)
        N = 0; % num nodes in path
        nodes = [];
    end
    
    methods
        % constructor
        function [obj] = Path(varargin)
            if nargin > 0
                nodes = varargin{1};
                obj.N = length(nodes);
                obj.nodes = nodes;
                obj.linkAllNodes();
            end
        end
        
        % link paths in order
        function linkAllNodes(obj)
            for i=1:obj.N
                if i > 1
                    obj.nodes(i).linkIncomingNode(obj.nodes(i-1));
                end
                
                if i < obj.N
                    obj.nodes(i).linkOutgoingNode(obj.nodes(i+1));
                end
            end
        end
        
        % add a single node
        function addNodeToEnd(obj, node)
            obj.nodes = [obj.nodes node];
            obj.N = obj.N + 1;
            if obj.N > 1
                obj.nodes(obj.N).linkIncomingNode(obj.nodes(obj.N-1));
                obj.nodes(obj.N-1).linkOutgoingNode(obj.nodes(obj.N));
            end
        end
        
        % get x and y coordinate of all nodes for plotting
        function [xarr,yarr] = getNodeCoordinates(obj)
            xarr = zeros(1,obj.N);
            yarr = zeros(1,obj.N);
            
            for i=1:obj.N
                xarr(i) = obj.nodes(i).xy(1);
                yarr(i) = obj.nodes(i).xy(2);
            end
        end
        
        % get paths connecting nodes for plotting
        function [E] = getPathEdges(obj)
            E = zeros(obj.N-1 , 2, 2); % #Edges, x&y, start&fin
            for i=1:( obj.N - 1)
                [ex,ey] = obj.nodes(i).getOutgoingEdge();
                E(i,1,:) = ex;
                E(i,2,:) = ey;
            end
        end
        
        % respond to forces on path
        function moveNodesAccordingToForce(obj, dt)
            for i=1:obj.N
                obj.nodes(i).moveAccordingToForce(dt);
            end
        end
        
        
    end
    
end


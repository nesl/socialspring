classdef PathNode < handle
    %PATHNODE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (GetAccess = private)
        GRID_X
        GRID_Y
        angle_init
        xy_init
        anchors_xy = [
            0 0
            0 0
            0 0
            0 0
            ];
        
        k_angle = 0;
        k_world = 0; % up/down/left/right
        k_incoming = 0;
        k_outgoing = 0;
        k_additional = [];
        IncomingNode = [];
        OutgoingNode = [];
        AdditionalNodes = [];
        AdditionalDistances = [];
        damping
        vel
        
        % a binary map for walls and obstructions
        map
    end
    
    properties (GetAccess = public)
        xy
        angle
    end
    
    methods
        %% constructor
        function obj = PathNode(xy, gridx, gridy)
            % assign node properties
            obj.xy_init = xy;
            obj.xy = xy;
            obj.GRID_X = gridx;
            obj.GRID_Y = gridy;
            
            % anchor this node in world
            %    - above
            obj.anchors_xy(1,:) = [xy(1) obj.GRID_Y];
            %    - from left
            obj.anchors_xy(2,:) = [0 xy(2)];
            %    - below
            obj.anchors_xy(3,:) = [xy(1) 0];
            %    - right
            obj.anchors_xy(4,:) = [obj.GRID_X xy(2)];
            
            % set initial velocity equal to 0
            obj.vel = 0;
            
            % set spring damper to 0
            obj.damping = 0;
            
            % initialize map
            obj.map = BinaryMap(gridx, gridy);
        end
        
        %% Set Map
        function setMap(obj, newmap)
            obj.map = newmap;
        end
        
        %% Set linked nodes
        % TODO: initialize angle here
        function linkIncomingNode(obj, h_node)
            obj.IncomingNode = h_node;
            if ~isempty(obj.OutgoingNode)
                obj.fixAngle();
            end
        end
        
        function linkOutgoingNode(obj, h_node)
            obj.OutgoingNode = h_node;
            if ~isempty(obj.IncomingNode)
                obj.fixAngle();
            end
        end
        
        function constrainToNode(obj, h_node, dist, k)
            % first this node
            obj.addConstraint(h_node, dist, k);
            % and then the other node
            h_node.addConstraint(obj, dist, k);
        end
        
        % this function is called from "constrainToNode". we need to
        % separate so that we avoid infinite recursion.
        function addConstraint(obj, h_node, dist, k)
            obj.AdditionalNodes = [obj.AdditionalNodes
                h_node];
            obj.k_additional = [obj.k_additional
                k];
            obj.AdditionalDistances = [obj.AdditionalDistances
                dist];
        end
        
        %% Get angle stressor
        function [stress] = getAngleStress(obj)
            if isempty(obj.IncomingNode) || isempty(obj.OutgoingNode)
                stress = 0;
            else
                stress = obj.k_angle*(obj.angle - obj.angle_init);
            end
        end
        
        %% Get linked nodes
        function [node] = getIncomingNode(obj)
            node =  obj.IncomingNode;
        end
        
        function [node] = getOutgoingNode(obj)
            node = obj.OutgoingNode;
        end
        
        %% Set spring constants
        function setDamping(obj,damp)
            % set node damping (should really be per spring, but hey)
            obj.damping = damp;
        end
        
        function setIncomingCertainty(obj,cert)
            % set "from" spring constant
            obj.k_incoming = cert;
        end
        
        function setOutgoingCertainty(obj,cert)
            % set "to" spring constant
            obj.k_outgoing = cert;
        end
        
        function setAbsoluteCertainty(obj,cert)
            % set "world" spring constants
            obj.k_world = cert;
        end
        
        function setAngleCertainty(obj,cert)
            % set the certainty about the angle incident on this node
            obj.k_angle = cert;
        end
        
        %% Get the current force vector
        function [force] = getForceVector(obj)
            % initialize force vector
            force = [0 0];
            epsilon = 1e-6;
            
            % force vector : "World" nodes
            for i=1:4
                vec = obj.anchors_xy(i,:) - obj.xy;
                vec_normal = (vec+epsilon)./( sum(abs(vec)) + epsilon);
                stretch = norm(vec,2) - ...
                    norm(obj.xy_init - obj.anchors_xy(i,:), 2);
                % aggregate spring force
                force = force + obj.k_world*vec_normal*stretch;

            end
            
            
            % force vector : "Incoming" node
            if ~isempty(obj.IncomingNode)
                vec = obj.IncomingNode.xy - obj.xy;
                vec_normal = (vec+epsilon)./( sum(abs(vec)) + epsilon);
                stretch = norm(vec,2) - ...
                    norm(obj.xy_init - obj.IncomingNode.xy_init, 2);
                % aggregate spring force
                force = force + obj.k_incoming*vec_normal*stretch;
            end
            
            % force vector : "Outgoing" node
            if ~isempty(obj.OutgoingNode)
                vec = obj.OutgoingNode.xy - obj.xy;
                vec_normal = (vec+epsilon)./( sum(abs(vec)) + epsilon);
                stretch = norm(vec,2) - ...
                    norm(obj.xy_init - obj.OutgoingNode.xy_init, 2);
                % aggregate spring force
                force = force + obj.k_outgoing*vec_normal*stretch;
            end
            
            % force vector : "Additional" nodes
            for i=1:length(obj.AdditionalNodes)
                vec = obj.AdditionalNodes(i).xy - obj.xy;
                vec_normal = (vec+epsilon)./( sum(abs(vec)) + epsilon);
                stretch = norm(vec,2) - obj.AdditionalDistances(i);
                % aggregate spring force
                force = force + obj.k_additional(i)*vec_normal*stretch;
            end
            
            %{
            % force vector : Incoming Angle spring
            if ~isempty(obj.IncomingNode)
                % find the force vector, perp to the edge (r.h.r.)
                vec = obj.xy - obj.IncomingNode.xy;
                vec_norm = vec/sqrt(norm(vec));
                vec_perp = [0 1; -1 0]*vec_norm'; % - 90 deg
                % find the change in angle w.r.t. initial condition
                angular_stress = obj.IncomingNode.getAngleStress();
                % add angular force
                % TODO
                force = force + angular_stress*vec_perp';
            end
            
            % force vector : Outgoing Angle spring
            if ~isempty(obj.OutgoingNode)
                % find the force vector, perp to the edge (r.h.r.)
                vec = obj.xy - obj.OutgoingNode.xy;
                vec_norm = vec/sqrt(norm(vec));
                vec_perp = [0 -1; 1 0]*vec_norm'; % + 90 deg
                % find the change in angle w.r.t. initial condition
                angular_stress = obj.OutgoingNode.getAngleStress();
                % add angular force
                % TODO
                force = force + angular_stress*vec_perp';
            end
            %}            
            
        end
        
        %% Motion Functions
        function moveAccordingToForce(obj, deltaT)
            % get acting force vector
            forcevec = obj.getForceVector();
            % get damping foce
            forcedamp = -obj.vel*obj.damping;
            % update velocity
            obj.vel = obj.vel + deltaT*(forcevec + forcedamp);
            % update position, according to map
            obj.xy = obj.map.traverseMap(obj.xy, obj.vel);
            
            % update my angle if i'm not an endpoint
            if ~isempty(obj.IncomingNode) && ~isempty(obj.OutgoingNode)
                vec_in = obj.xy - obj.IncomingNode.xy;
                vec_out = obj.OutgoingNode.xy - obj.xy;
                if all(vec_in == 0) || all(vec_out == 0)
                    obj.angle = [];
                else
                    vec_in = vec_in./sqrt(norm(vec_in));
                    vec_out = vec_out./sqrt(norm(vec_out));
                    
                    obj.angle = obj.angle_init;
                end
            end
            
        end
        
        %% Miscellanious Utility Functions
        function [edgex, edgey] = getOutgoingEdge(obj)
            if isempty(obj.OutgoingNode)
                edgex = [];
                edgey = [];
            else
                edgex = [obj.xy(1) obj.OutgoingNode.xy(1)];
                edgey = [obj.xy(2) obj.OutgoingNode.xy(2)];
            end
        end
        
        function fixAngle(obj)
            if ~isempty(obj.IncomingNode) && ~isempty(obj.OutgoingNode)
                vec_in = obj.xy - obj.IncomingNode.xy;
                vec_out = obj.OutgoingNode.xy - obj.xy;
                if all(vec_in == 0) || all(vec_out == 0)
                    obj.angle_init = [];
                    obj.angle = [];
                else
                    vec_in = vec_in./sqrt(norm(vec_in));
                    vec_out = vec_out./sqrt(norm(vec_out));
                    
                    obj.angle_init = atan2(vec_out(2), vec_out(1)) - atan2(vec_in(2),vec_in(1));
                    obj.angle = obj.angle_init;
                end
            end
        end
        
        function [force] = getAngleInForce(obj)
            % force vector : Incoming Angle spring
            if ~isempty(obj.IncomingNode)
                % find the force vector, perp to the edge (r.h.r.)
                vec = obj.xy - obj.IncomingNode.xy;
                vec_norm = vec/sqrt(norm(vec));
                vec_perp = [0 1; -1 0]*vec_norm'; % - 90 deg
                % find the change in angle w.r.t. initial condition
                angular_stress = obj.IncomingNode.getAngleStress();
                % add angular force
                % TODO
                force = angular_stress*vec_perp';
            else
                force = [];
            end
        end
        
        function [force] = getAngleOutForce(obj)
            % force vector : Outgoing Angle spring
            if ~isempty(obj.OutgoingNode)
                % find the force vector, perp to the edge (r.h.r.)
                vec = obj.xy - obj.OutgoingNode.xy;
                vec_norm = vec/sqrt(norm(vec));
                vec_perp = [0 -1; 1 0]*vec_norm'; % + 90 deg
                % find the change in angle w.r.t. initial condition
                angular_stress = obj.OutgoingNode.getAngleStress();
                % add angular force
                % TODO
                force = angular_stress*vec_perp';
            else
                force = [];
            end
        end
        
    end
    
end


%% Housekeeping
clc; close all; clear all;

%% Add paths
addpath('classes');

%% Seed PRNG
rng(103);

%% Customize environment
% environment boundaries
xlimits = [0 50]; % meters
ylimits = [0 50]; % meters
GRID_X = xlimits(2) - xlimits(1);
GRID_Y = ylimits(2) - ylimits(1);
% simulation length and granularity
sim_end = 50; % seconds
sim_step = 0.25; % seconds
% distance to snap agent to waypoint
snap_dist = 0.1; % meters
% what is the maximum distance apart two agents can see (encounter) each
% other?
encounter_max_dist = 15; % meters
% max. encounters for the simulation, to avoid growing an array
MAX_ENCOUNTERS = 100000;
% noise for estimating the agent positions (like dead-reckoning). This is
% gaussian noise that is integrated once to give the estimation error
estimation_noise_variance = .01;

%% Plot options
PLOT_AGENTS = true;
PLOT_ESTIMATES = true;
PLOT_ESTIMATE_LINKS = true;
PLOT_ENCOUNTERS = true;
SAVE_MOVIE = false;


%% Create agents
% number of agents to use
N_agents = 10;
% average time an agent will stay in one place
mean_dwell_time = 10; % seconds (1/lambda for poisson)

% agent coordinate matrix
agents_coords = [GRID_X*rand(N_agents,1) GRID_Y*rand(N_agents,1)]; % (x,y)
% agent coordinate estimate matrix
agents_estimates = agents_coords; % start with perfect estimation
% accumulating error in estimating agent coordinates (for integrating)
agents_errors = zeros(size(agents_coords));
% agent waypoint matrix--random destinations for each agent
agents_waypoints = agents_coords;
% how much time is left for each agent to stay where they are
agents_waits = exprnd(mean_dwell_time, N_agents, 1);
% how fast is each agent? This doesn't change per agent right now. 1.1 mps
% is average speed of a human walking.
agents_speeds = max( 1.1 + 0.5*randn(N_agents,1) , 0 );
% matrix of agent encounters: -1 if none, distance otherwise. Lower
% triangular.
agents_encounters = -1*ones(N_agents, N_agents);

% keep track of historical positions for correcting and plotting
agents_coords_history =    zeros(ceil(sim_end/sim_step), N_agents, 2); % x, y
agents_estimates_history = zeros(ceil(sim_end/sim_step), N_agents, 2); % x, y
agents_encounter_counts = zeros(N_agents, 1);
agents_encounter_history = zeros(N_agents, MAX_ENCOUNTERS, 3); % iter, node, dist

%% Set up movie
if SAVE_MOVIE
    vidObj = VideoWriter('outputs/EncounterMovie2.avi');
    vidObj.FrameRate=30;
    open(vidObj);
end

%% Plot the environment
fig = cfigure(20,20);
set(gcf,'color','w');
hold on;
if PLOT_AGENTS
    h_agents = plot(agents_coords(:,1), agents_coords(:,2), 'ob','MarkerFaceColor','b',...
        'MarkerSize',12);
end
if PLOT_ESTIMATES
    h_estimates = plot(agents_coords(:,1), agents_coords(:,2), 'xr','MarkerFaceColor','b',...
        'MarkerSize',12, 'LineWidth',2);
end
if PLOT_ESTIMATE_LINKS
    H_LINKS = zeros(N_agents,1);
    for i=1:N_agents
        H_LINKS(i) = plot([agents_coords(i,1) agents_estimates(i,1)],...
            [agents_coords(i,2) agents_estimates(i,2)], '--k');
    end
end
if PLOT_ENCOUNTERS
    encounter_reset = [-100 -100 -100 -100];
    H_ENCOUNTERS = zeros(N_agents,N_agents);
    for i=1:N_agents
        for j=1:N_agents
            H_ENCOUNTERS(i,j) = plot(encounter_reset(1:2), encounter_reset(3:4), ...
                '-r');
        end
    end
end
grid on;
xlabel('X Position','FontSize',16);
ylabel('Y Position','FontSize',16);
xlim(xlimits);
ylim(ylimits);

%% Simulation
iter = 0;

% iterate over time
for t= 0:sim_step:sim_end
    
    % increment iteration count
    iter = iter + 1;
    
    % append agent locations to history
    agents_coords_history(iter,:,:) = agents_coords;
    
    % append new estimate to history
    agents_estimates_history(iter,:,:) = agents_estimates;
    
    % calculate distance from agents to destinations
    agents_distances = sqrt( sum( (agents_waypoints - agents_coords).^2 , 2) );
    % find those agents that are no longer moving
    agents_stopped = find(agents_distances <= snap_dist);
    % agents that have reached destination have waited 'sim_step' longer
    agents_waits(agents_stopped) = agents_waits(agents_stopped) - sim_step;
    
    % ------- move agents that need to be moved -------
    % find vector to dest.
    agents_vectors = agents_waypoints - agents_coords;
    % normalize vector (unless we're already at our destination)
    if ~isempty(find(agents_vectors(:,1) ~= 0))
        agents_vectors(agents_vectors(:,1) ~= 0, :) =...
            agents_vectors(agents_vectors(:,1) ~= 0, :)./...
            repmat(sqrt(sum( agents_vectors(agents_vectors(:,1) ~= 0, :).^2 ,2)), 1, 2);
        % find how much each agent 'should' move, given speed and vector
        delta_coords = repmat(sim_step*agents_speeds, 1, 2).*agents_vectors;
        
        for i=1:N_agents
            % distance of agent step
            agent_step_size = agents_speeds(i)*sim_step;
            % snap to waypoint if step is > distance remaining
            if agent_step_size > agents_distances(i)
                agents_coords(i,:) = agents_waypoints(i,:);
            else
                agents_coords(i,:) = agents_coords(i,:) + ...
                    delta_coords(i,:);
                
                % take new estimate measurement (only when moving)
                % get new measurement noise
                xr = randn*estimation_noise_variance;
                yr = randn*estimation_noise_variance;
                % keep track of cumulative sum
                agents_errors(i,:) = [agents_errors(i,1)+xr agents_errors(i,2)+yr];
                % add noise and new step to estimate
                agents_estimates(i,:) = agents_estimates(i,:) + ...
                    delta_coords(i,:) + sim_step*agents_errors(i,:); % normalize by time step
            end
            
            % -- check for encounters --
            for j=1:i
                % ignore "self-encounters"
                if j == i
                    continue;
                end
                % distance to other agent
                agent_distance = sqrt(sum( (agents_coords(j,:) - agents_coords(i,:)).^2 ));
                if agent_distance <= encounter_max_dist
                    % if distance is low enough, update encounter info.
                    agents_encounters(i,j) = agent_distance;
                    agents_encounters(j,i) = agent_distance;
                    
                    % increment number of encounters per agent
                    agents_encounter_counts(i) = agents_encounter_counts(i) + 1;
                    agents_encounter_counts(j) = agents_encounter_counts(j) + 1;
                    
                    % append to history
                    agents_encounter_history(i, agents_encounter_counts(i), :) = ...
                        [iter j agent_distance];
                    agents_encounter_history(j, agents_encounter_counts(j), :) = ...
                        [iter i agent_distance];
                    
                    
                    if PLOT_ENCOUNTERS
                        set(H_ENCOUNTERS(i,j),'XData', [agents_coords(i,1) agents_coords(j,1)],...
                            'YData', [agents_coords(i,2) agents_coords(j,2)]);
                    end
                    
                else
                    % reset encounters to "non-encounter" value: -1
                    agents_encounters(i,j) = -1;
                    agents_encounters(j,i) = -1;
                    
                    if PLOT_ENCOUNTERS
                        set(H_ENCOUNTERS(i,j),'XData', encounter_reset(1:2),...
                            'YData', encounter_reset(3:4));
                    end
                    
                end
            end
            
        end
    end
    
    
    % -- pick new waypoints --
    
    % which agents are ready to move?
    agents_moving = find(agents_waits <= 0);
    N_moving = size(agents_moving,1);
    % pick new random waypoints for moving agents
    agents_waypoints(agents_moving,:) = [GRID_X*rand(N_moving,1) GRID_Y*rand(N_moving,1)];
    % recharge wait times, using Poisson r.v.
    agents_waits(agents_moving) = exprnd(mean_dwell_time, N_moving, 1);
    
    % update plot
    if PLOT_AGENTS
        set(h_agents,'XData', agents_coords(:,1), 'YData', agents_coords(:,2));
    end
    if PLOT_ESTIMATES
        set(h_estimates,'XData', agents_estimates(:,1), 'YData', agents_estimates(:,2));
    end
    if PLOT_ESTIMATE_LINKS
        for i=1:N_agents
            set(H_LINKS(i), 'XData',[agents_coords(i,1) agents_estimates(i,1)],...
                'YData', [agents_coords(i,2) agents_estimates(i,2)]);
        end
    end
    the_title = sprintf('Iteration: %d / %d', iter, sim_end/sim_step);
    title(the_title, 'FontSize',16);
    
    
    % append to movie
    if SAVE_MOVIE
        f = getframe(fig);
        writeVideo(vidObj,f);
    end
    
    
    % slowdown factor
    pause(0.01);
    
    
end

if SAVE_MOVIE
    close(vidObj);
end

%% Downsample agent paths and encounters to be realistic
sample_rate = 3; % seconds

% create new arrays for downsampled versions
% keep track of historical positions for correcting and plotting
sampled_coordinates =    zeros(ceil(sim_end/sample_rate), N_agents, 2); % x, y
sampled_estimates = zeros(ceil(sim_end/sample_rate), N_agents, 2); % x, y
sampled_encounters = zeros(N_agents, MAX_ENCOUNTERS, 3); % iter, node, dist
sampled_encounter_counts = zeros(N_agents, 1);

% simulation time points
sim_times = 0:sim_step:sim_end;

% downsampled iterations
iter = 1;

% loop through and save points in increments of 'sample_rate'
for ts = 0:sample_rate:sim_end
    
    % find iteration corresponding to time closest (but <) to ts
    sim_idx = find(sim_times <= ts, 1, 'last');
    sampled_coordinates(iter, :, :) = squeeze( agents_coords_history(sim_idx, :, :) );
    sampled_estimates(iter, :, :) = squeeze( agents_estimates_history(sim_idx, :, :) );
    
    for i=1:N_agents
        num_encounters = agents_encounter_counts(i);
        encounter_idxs = find(agents_encounter_history(i, 1:num_encounters, 1) == sim_idx);
        if isempty(encounter_idxs)
            continue;
        end
        % get sampled encounter information
        samples = agents_encounter_history(i,encounter_idxs, :);
        sampled_count = sampled_encounter_counts(i);
        % change encounter index info to correspond to downsampling
        samples(1, :, 1) = iter;
        sampled_encounters(i, (sampled_count+1):(sampled_count + size(samples,2)), :) = samples;
        sampled_encounter_counts(i) = sampled_count + size(samples,2);
    end
    
    iter = iter + 1;
    
end

%% Create the path objects for the estimations
% initialize path object array
PathArray = [];

for agent_id=1:N_agents
    % get estimated coordinates
    coords = squeeze( sampled_estimates(:, agent_id, :) );
    
    p = Path();
    for i=1:size(coords,1)
        xy = coords(i,:);
        node = PathNode(xy, GRID_X, GRID_Y);
        node.setDamping(30);
        
        % set initial anchor and subsequent certainties
        stiffness = 10;
        if i == 1
            node.setAbsoluteCertainty(20);
        else
            node.setAbsoluteCertainty(stiffness/i);
        end
        
        % set relative distance certainties
        node.setOutgoingCertainty(2);
        node.setIncomingCertainty(2);
        
        % set relative angle certainties
        node.setAngleCertainty(0.5);
        
        % pop the node onto the end of the path object
        p.addNodeToEnd(node);
    end
    
    % append path object to array
    PathArray = [PathArray; p];
    
end

% Add the encounter constraints
for agent_id=1:N_agents
    num_encounters = sampled_encounter_counts(agent_id);
    encounters = squeeze( sampled_encounters( agent_id , 1:num_encounters, :) );
    
    for idx=1:num_encounters
        % extract encounter info
        iter = encounters(idx,1);
        target_id = encounters(idx,2);
        dist = encounters(idx,3);
        
        % add constraint
        k_encounter = 5;
        encounter_prob = 1;
        target_node = PathArray(target_id).nodes(iter);
        if rand < encounter_prob
            PathArray(agent_id).nodes(iter).constrainToNode(target_node, dist, k_encounter);
        end
        
    end
end

%% Distributed Spring GRP Solver
% plot handles
path_handles = zeros(1,N_agents);
colors = hsv(N_agents);

% Set up plot
fig = cfigure(15,15);
set(gcf,'color','w');
hold on;

for i=1:N_agents
    path_handles(i) = plot([0], [0], 'o-', 'MarkerSize', 10, 'Color', colors(i,:), 'MarkerFaceColor', colors(i,:));
end

grid on;
xlim([0 GRID_X]);
ylim([0 GRID_Y]);
xlabel('X Position','FontSize',14);
ylabel('Y Position','FontSize',14);
h_title = title('Time = 0 sec','FontSize',15);

if SAVE_MOVIE
    vidObj = VideoWriter('outputs/estimateCorrection2.avi');
    vidObj.FrameRate=30;
    open(vidObj);
end


%% Run simulation
FRAMERATE = 120;
dt = (1/FRAMERATE);
t_end = 0.5; % seconds

for t=0:dt:t_end
    
    % pause for plotting
    pause(.01);
    
    % dynamics
    for i=1:N_agents
        PathArray(i).moveNodesAccordingToForce(dt);
    end
    
    % update node plot locations
    for i=1:N_agents
        [x,y] = PathArray(i).getNodeCoordinates();
        set(path_handles(i), 'XData', x, 'YData', y);
    end

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


%% Calculate statistics
errors_before = [];
errors_after = [];

for agent_id=1:N_agents
    est_coords = squeeze( sampled_estimates(:, agent_id, :) );
    tru_coords = squeeze( sampled_coordinates(:, agent_id, :) );
    [x,y] = PathArray(agent_id).getNodeCoordinates();
    fix_coords = [x' y'];
    
    for i=1:size(est_coords,1)
        e_before = norm(est_coords(i,:) - tru_coords(i,:));
        e_after = norm(fix_coords(i,:) - tru_coords(i,:));
        
        errors_before = [errors_before; e_before];
        errors_after = [errors_after; e_after];
    end
    
end


cfigure(10,10);
plot(sort(errors_before,'descend'), 'o-b');
hold on;
plot(sort(errors_after,'descend'), 's-r');
grid on;
xlabel('Sorted node ID','FontSize',14);
ylabel('Error (distance)','FontSize',14);
legend('Before Refinement','After Refinement','Location','NE');
saveplot('figures/error_correction2_improve_square');

fprintf('from E = %.2f to E = %.2f\n (%.2f %%)\n',...
    mean(errors_before), mean(errors_after),...
    100*( mean(errors_after) - mean(errors_before) )/mean(errors_before));













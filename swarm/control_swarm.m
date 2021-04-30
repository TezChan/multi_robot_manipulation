clear all; close all;
%--------------------------------------------------------------------------
% setup environment
%--------------------------------------------------------------------------
colobj = setup_environment();
width = 12;
height = 12;
res = 100;
N = 1000;
occ_map = create_occupancy(width, height, res, N, colobj);
map_points = [];
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% setup scanner
%--------------------------------------------------------------------------
M = 16;
range_scanner = 10;
R_scanner = 0.1 * eye(M);
scanner = Scanner(M, range_scanner, R_scanner);
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% setup observation sensors
%--------------------------------------------------------------------------
range_range = 8;
R_range = 0.1;
R_bearing = 0.1;
sensors{1} = RangeSensor([0;0;4], range_range, R_range);
sensors{2} = RangeSensor([10;0;4], range_range, R_range);
sensors{3} = RangeSensor([0;10;4], range_range, R_range);
sensors{4} = RangeSensor([10;10;4], range_range, R_range);
sensors{5} = BearingSensor([8;5;.75], R_bearing);
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% setup mobile robot agents
%--------------------------------------------------------------------------
num_agents = 3;
P = 0.1 * eye(3);
Q = 0.001 * eye(3);
rob_radius = 0.2;
rob_height = 0.2;
rob_mass = 1;
is_omni = true;
free_space = generate_free_space(rob_radius, [0 10 0 10], 50, occ_map);
rand_pos = randperm(size(free_space, 1), num_agents);
for ii=1:num_agents
    q0 = [free_space(rand_pos(ii), :)'; 2 * pi * rand(1, 1)];
    agents(ii) = MobileRobot(q0, P, Q, rob_radius, rob_height, rob_mass, is_omni, sensors, scanner);
end
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% setup agent communication network
%--------------------------------------------------------------------------
s = 1:num_agents;
t = [2:num_agents, 1];
G = graph(s, t);
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% setup swarm
%--------------------------------------------------------------------------
swarm_position = [2; 2];
swarm_radius = 0.75;
swarm_angle = 0;
rel_scheme = 1;
pos_scheme = 1;
platoon_sep = 1.5;
swarm = Swarm(swarm_position, swarm_radius, swarm_angle, agents, G, rel_scheme, pos_scheme, platoon_sep);
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% simulation
%--------------------------------------------------------------------------
sim = figure(1);
clf(sim);
show(occ_map);
view(-90, 90);
hold on;

% used to manually quit simulation
key = ' ';
key_pressed = 0;
set(sim,'KeyPressFcn',@get_key);

% setup video
video_writer = VideoWriter('./videos/test.avi');
video_writer.FrameRate = 10;
open(video_writer);
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% setup path planning
%--------------------------------------------------------------------------
epsilon = 0.04;

pos_final = [8 8];

is_teleop = true;

% index of desired position along path
path_ind = 0;

% size of path
path_size = 0;
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% control swarm
%--------------------------------------------------------------------------
% time step
ts = 1;

% distance used for collision avoidance
col_buffer = 0.3;

% control input saturation
u_max = 0.2 * ones(swarm.num_features, 1);

% initial states of agents
agent_states = get_agent_states(swarm);

% setup plots
hold on;
agent_plot = scatter(agent_states(1,:), agent_states(2,:), 90, hsv(swarm.num_agents));
swarm_plot = plot(0, 0, 'rx', 'MarkerSize', 10);
path_plot = plot(0, 0, 'b', 'LineWidth', 2);
hold off;

% capture simulation frame
frame = getframe(sim);
writeVideo(video_writer, frame);

while key ~= 'q' && path_ind <= path_size    
    % control swarm
    swarm = control(swarm, colobj, col_buffer, u_max, ts, epsilon);
    agent_states = get_agent_states(swarm);
    
    if swarm.phase == 2
        % path following phase
        if path_size == 0
            % need to create a path
            if is_teleop
                switch swarm.pos_scheme
                    case 1
                        % distributed (centroid) control
                        path = get_agent_centroid(swarm);
                    case 2
                        % follow-the-leader control
                        path = agent_states(1:2,1);
                end
                path_size = 1;
                path_ind = 1;
            else
                switch swarm.pos_scheme
                    case 1
                        % distributed (centroid) control
                        swarm_position = get_agent_centroid(swarm);
                    case 2
                        % follow-the-leader control
                        swarm_position = agent_states(1:2,1);
                end
                num_sample = floor(0.25 * size(free_space, 1));
                num_neighbors = 4;
                free_space_swarm = generate_free_space(swarm.radius, [0 10 0 10], 50, occ_map);
                [vertices, edges, path] = probability_road_map(free_space_swarm, occ_map, swarm_position', pos_final, num_sample, num_neighbors);
                path = path';
                path_size = size(path, 2);
                path_ind = 1;
            end
        end
        
        if is_teleop && key_pressed
            % update swarm position from keyboard
            swarm = teleop(swarm, key);
            path = [path, swarm.position];
            path_size = path_size + 1;
            path_ind = path_size;
            key_pressed = 0;
        end
        
        % update swarm positioning
        swarm_position = path(:,path_ind);
        swarm = set_position(swarm, swarm_position);
        
        % update simulation
        swarm_plot.XData = swarm_position(1);
        swarm_plot.YData = swarm_position(2);
        path_plot.XData = path(1,:);
        path_plot.YData = path(2,:);
    end
    
    % desired position reached
    if ~is_teleop && swarm.converged
        path_ind = path_ind + 1;
    end
    
    % update simulation
    agent_plot.XData = agent_states(1,:);
    agent_plot.YData = agent_states(2,:);
    
    % capture simulation frame
    frame = getframe(sim);
    writeVideo(video_writer, frame);
end

close(video_writer);
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% purpose: get pressed key
%--------------------------------------------------------------------------
function get_key(~, event)
assignin('base', 'key', event.Key);
assignin('base', 'key_pressed', 1);
end
%--------------------------------------------------------------------------
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
% setup mobile robot
%--------------------------------------------------------------------------
P = 0.1 * eye(3);
Q = 0.001 * eye(3);
q0 = [1; 2; 0];
radius_rob = 0.2;
height_rob = 0.2;
mass_rob = 1;
is_omni = false;
robot = MobileRobot(q0, P, Q, radius_rob, height_rob, mass_rob, is_omni, sensors, scanner);
free_space = generate_free_space(radius_rob, [0 10 0 10], 50, occ_map);
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% setup simulation figure
%--------------------------------------------------------------------------
sim = display_simulation(robot, colobj, 1);
pause(1);

% used to manually quit simulation
key = ' ';
key_pressed = 0;
set(sim,'KeyPressFcn',@get_key);

% setup video
video_writer = VideoWriter('./videos/test.avi');
video_writer.FrameRate = 10;
open(video_writer);

% capture simulation frame
frame = getframe(sim);
writeVideo(video_writer, frame);
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% setup path planning
%--------------------------------------------------------------------------
path_epsilon = 0.1;

pos_final = [8 8];

alg = 'brush fire';

switch alg
    case 'bug 0'
        path = pos_final';
        vertices = [];
        edges = [];
    case 'potential field'
        path = pos_final';
        vertices = [];
        edges = [];
        gain_att = 1;
        gain_rep = 1;
        col_buffer = 0.5;
        distance_thresh = 5;
        alpha = 0.1;
    case 'brush fire'
        path = pos_final';
        vertices = [];
        edges = [];
        res = 3;
        brush_fire_grid = nan(height * res, width * res);
        brush_fire_grid = update_brush_fire_grid(brush_fire_grid, height, width, res, pos_final(2), pos_final(1), 0);
    case 'prm'
        num_sample = floor(0.25 * size(free_space, 1));
        num_neighbors = 4;
        [vertices, edges, path] = probability_road_map(free_space, occ_map, robot.state_est(1:2)', pos_final, num_sample, num_neighbors);
        path = path';
    case 'rrt'
        num_sample = floor(0.25 * size(free_space, 1));
        [vertices, edges, path] = rapidly_exploring_random_tree(free_space, occ_map, robot.state_est(1:2)', pos_final, num_sample);
        path = path';
end

% index of desired position along path
path_ind = 1;
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% control robot
%--------------------------------------------------------------------------
ts = 1;
max_linear = 0.1;
max_angular = 0.5;

true_hist = robot.state_true;
est_hist = robot.state_est;

while ~strcmp(key, 'q') && path_ind <= size(path, 2)
    % use robot's scanner
    [new_points, z_scanner, ang_scanner] = scan(robot, occ_map);
    map_points = [map_points; new_points];
    
    % create list of control inputs
    switch alg
        case 'bug 0'
            col_buffer = 7.5 / ts * max_linear;
            u_list = bug0(robot.state_est, path(:, path_ind), z_scanner, ang_scanner, col_buffer, max_linear, max_angular);
        case 'potential field'
            u_list = potential_field(robot.state_est, path(:, path_ind), new_points, max_linear, max_angular,...
                gain_att, gain_rep, col_buffer, distance_thresh, alpha);
        case 'brush fire'
            [u_list, brush_fire_grid] = brush_fire(robot.state_est, brush_fire_grid, height, width, res, new_points, max_linear, max_angular);
        case {'prm', 'rrt'}
            col_buffer = 0;
            u_list = bug0(robot.state_est, path(:, path_ind), z_scanner, ang_scanner, col_buffer, max_linear, max_angular);
    end
    
    % apply control inputs to robot
    for ii=1:size(u_list,2)
        u = u_list(:,ii);
        
        robot = kalman_filter(robot, u, ts);

        true_hist = [true_hist, robot.state_true];
        est_hist = [est_hist, robot.state_est];

        figure(sim);
        delete(sim.CurrentAxes.Children(1));
        [~,patchObj] = show(robot.col_body);
        patchObj.FaceColor = [1 0 0];
        patchObj.EdgeColor = 'none';
        drawnow;
        
        % capture simulation frame
        frame = getframe(sim);
        writeVideo(video_writer, frame);
        
        % desired position reached
        if norm(abs(robot.state_est(1:2) - path(:, path_ind))) < path_epsilon
            path_ind = path_ind + 1;
            break;
        end
    end
end
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% plot results
%--------------------------------------------------------------------------
sim = plot_path_analysis(true_hist, est_hist, pos_final, free_space, occ_map,...
    vertices, edges, path, alg, sim);

% capture simulation frame
frame = getframe(sim);
writeVideo(video_writer, frame);

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
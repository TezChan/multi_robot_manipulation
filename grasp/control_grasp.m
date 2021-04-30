clear all; close all;
rng(0);
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
P = 1e-10 * eye(3);
Q = 1e-10 * eye(3);
rob_radius = 0.2;
rob_height = 0.2;
rob_mass = 5;
is_omni = true;
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% setup grasp
%--------------------------------------------------------------------------
grasp_init = [0; 2; 2];
grasp_final = [pi/2; 8; 8];

grasp_height = 1;
grasp_width = 0.5;
load_mass = 10;
grasp_type = 3;
phi = deg2rad(45);

ex = [1; 0; 0];
ey = [0; 1; 0];
ec = [ey, -ey, ex, -ex];
ea = ec;
R0C = rot([0;0;1], grasp_init(1));

contact_offsets = [grasp_height / 2; grasp_width / 2; 0] .* ec;
agent_offsets = rob_radius * ea;

for ii = 1:size(contact_offsets, 2)
    agents(ii) = MobileRobot([grasp_init(2:3); 0] + R0C * (contact_offsets(:,ii) + agent_offsets(:,ii)),...
        P, Q, rob_radius, rob_height, rob_mass, is_omni, sensors, scanner);
end

grasp = RectangleGrasp(grasp_height, grasp_width, load_mass, grasp_type, phi, contact_offsets, agents);
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% simulation
%--------------------------------------------------------------------------
sim = figure(1);
clf(sim);
show(occ_map);
view(-90, 90);
hold on;

% setup video
video_writer = VideoWriter('test.avi');
video_writer.FrameRate = 10;
open(video_writer);
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% grasp manipulation
%--------------------------------------------------------------------------
free_space = generate_free_space(grasp, [0 10 0 10], 11, occ_map);
[path, path_smooth] = plan_path(grasp, free_space, occ_map, grasp_init', grasp_final', floor(0.25 * size(free_space, 1)), 8, 100);
[trajectory, t] = plan_trajectory(grasp, path_smooth, 0.5, 0.1, 0.1);
v_contacts = compute_contact_velocities(grasp, trajectory, t(2));
F_load = compute_load_force(grasp, trajectory, t(2));
F_contacts = compute_contact_forces(grasp, trajectory, F_load);
agent_states = get_agent_states(grasp);

% circle for agents
N = 100;
angles = 2 * pi * (0:N-1)' / N;
circ_def = rob_radius * [cos(angles), sin(angles)];
circs_x = circ_def(:,1) + agent_states(1,:);
circs_y = circ_def(:,2) + agent_states(2,:);

% rectangle for grasp
N = 5;
rect_x = linspace(-grasp.height / 2, grasp.height / 2, N)';
rect_y = linspace(-grasp.width / 2, grasp.width / 2, N)';
rect_top = [rect_x, rect_y(end) * ones(N, 1)];
rect_right = [rect_x(end) * ones(N,1), flip(rect_y)];
rect_bottom = [flip(rect_x), rect_y(1) * ones(N, 1)];
rect_left = [rect_x(1) * ones(N,1), rect_y];
rect_def = [rect_top; rect_right; rect_bottom; rect_left; rect_x(1), rect_y(end)];
angle = grasp_init(1);
rect = rect_def * [cos(angle) -sin(angle); sin(angle) cos(angle)]' + grasp_init(2:3)';

% setup plots
plot_path(grasp, path);
hold on;
agent_plot = plot(circs_x, circs_y);
trajectory_plot = plot(rect(:,1), rect(:,2), 'g');
for ii=1:grasp.num_agents
    p = [agent_states(1:2,ii), grasp_init(2:3) + R0C(1:2, 1:2) * contact_offsets(1:2,ii)];
    link_plots(ii) = plot(p(1,:), p(2,:), 'k');
end
hold off;

% capture simulation frame
frame = getframe(sim);
writeVideo(video_writer, frame);

for ii=1:numel(t)
    % current contact velocities
    v = v_contacts(ii,:);
    
    % control input
    u = compute_agent_velocities(grasp, v, trajectory(ii,:));
    
    % update grasp
    grasp = update_agent_states(grasp, u, t(2));
    agent_states = get_agent_states(grasp);
    
    % update simulation
    circs_x = circ_def(:,1) + agent_states(1,:);
    circs_y = circ_def(:,2) + agent_states(2,:);
    angle = trajectory(ii, 1);
    R0C = rot([0;0;1], angle);
    rect = rect_def * R0C(1:2, 1:2)' + trajectory(ii, 2:3);
    for jj=1:grasp.num_agents
        agent_plot(jj).XData = circs_x(:,jj);
        agent_plot(jj).YData = circs_y(:,jj);
        p = [agent_states(1:2,jj), trajectory(ii, 2:3)' + R0C(1:2, 1:2) * contact_offsets(1:2,jj)];
        link_plots(jj).XData = p(1,:);
        link_plots(jj).YData = p(2,:);
    end
    trajectory_plot.XData = rect(:,1);
    trajectory_plot.YData = rect(:,2);
    
    % capture simulation frame
    frame = getframe(sim);
    writeVideo(video_writer, frame);
end

close(video_writer);
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% plot results
%--------------------------------------------------------------------------
plot_force_analysis(F_contacts, F_load, t, 2);
%--------------------------------------------------------------------------
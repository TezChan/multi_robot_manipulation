%--------------------------------------------------------------------------
% purpose generate 2D free space for a circular robot
%  input:  robot_radius = radius of circular mobile robot
%                bounds = bounds of free space [xmin xmax ymin ymax]
%                     N = number of points
%               occ_map = 2D binary occupancy map
% output:    free_space = collection of valid 2D positions for the robot
%--------------------------------------------------------------------------
function [free_space] = generate_free_space(robot_radius, bounds, N, occ_map)
% x and y points to try
x = linspace(bounds(1), bounds(2), N);
y = linspace(bounds(3), bounds(4), N);

% create positions
[X, Y] = meshgrid(x, y);
positions = [X(:), Y(:)];
M = size(positions, 1);

% keep track of valid positions
valid_pos = true(M, 1);

% default robot occupancy 
ang = linspace(0,2*pi,N);
circ = robot_radius * [cos(ang)', sin(ang)'];

% determine valid positions
for ii=1:M
   collisions = checkOccupancy(occ_map, circ + positions(ii,:));
   valid_pos(ii) = ~any(collisions == 1);
end
free_space = positions(valid_pos, :);
end
%--------------------------------------------------------------------------
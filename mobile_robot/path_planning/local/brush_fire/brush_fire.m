%--------------------------------------------------------------------------
% purpose: use brush fire algorithm for path planning
%  input:       x_est = estimated state
%                grid = brush fire grid
%            height_m = height of brush fire grid in meters
%             width_m = width of brush fire grid in meters
%                 res = resolution of brush fire grid in cells per meter
%          map_points = 2D map points from scanner measurements
%          max_linear = maximum linear velocity
%         max_angular = maximum angular velocity
% output:           u = control input
%                grid = brush fire grid
%--------------------------------------------------------------------------
function [u, grid] = brush_fire(x_est, grid, height_m, width_m, res, map_points, max_linear, max_angular)
% update grid with map points from scanner
grid = update_brush_fire_grid(grid, height_m, width_m, res, map_points(:,2), map_points(:,1), inf(size(map_points,1)));

% dimensions of grid in cells
[height_cell, width_cell] = size(grid);

% codebook of y and x values
code_y = linspace(0, height_m, height_m * res);
code_x = linspace(0, width_m, width_m * res);

% partition of y and x values
part_y = code_y(1:end-1) + diff(code_y(1:end)) / 2;
part_x = code_x(1:end-1) + diff(code_x(1:end)) / 2;

% determine grid indices associated with estimated position
y_ind = quantiz(x_est(2), part_y) + 1;
x_ind = quantiz(x_est(1), part_x) + 1;

% linear index of estimated position
lin_ind = sub2ind([height_cell, width_cell], y_ind, x_ind);

% get neighbors to current position
neighbors = get_neighbors(lin_ind, height_cell, width_cell);

% neighbor values
neighbor_values = grid(neighbors);

% neighbor of smallest value
[~, ind_desired] = min(neighbor_values);
neighbor_lin = neighbors(ind_desired);
[neighbor_y, neighbor_x] = ind2sub([height_cell, width_cell], neighbor_lin);

% new position
pos_new = [code_x(neighbor_x); code_y(neighbor_y)];
delta_pos = pos_new - x_est(1:2);
ang_desired = mod(atan2(delta_pos(2), delta_pos(1)) - x_est(3), 2*pi);
dist_desired = norm(delta_pos);

% put desired angle in [-pi, pi)
if ang_desired > pi
    ang_desired = ang_desired - 2*pi;
end

% commands to rotate to desired angle
num_max_turn = floor(abs(ang_desired) / max_angular);
final_turn = rem(abs(ang_desired), max_angular);
u_ang = [0; max_angular];
u_ang = sign(ang_desired) * [repmat(u_ang, 1, num_max_turn), [0; final_turn]];

% command to move forward
num_max_forward = floor(dist_desired / max_linear);
final_forward = rem(dist_desired, max_linear);
u_lin = [max_linear; 0];
u_lin = [repmat(u_lin, 1, num_max_forward), [final_forward; 0]];

% control inputs
u = [u_ang, u_lin];
end
%--------------------------------------------------------------------------
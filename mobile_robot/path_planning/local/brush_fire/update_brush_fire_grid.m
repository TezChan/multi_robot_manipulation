%--------------------------------------------------------------------------
% purpose: update the brush fire grid
%  input:      grid = brush fire grid
%          height_m = height of brush fire grid in meters
%           width_m = width of brush fire grid in meters
%               res = grid resolution in cells per meters
%            y_list = list of y positions to update
%            x_list = list of x positions to update
%            values = list of values to update grid with
% output:      grid = brush fire grid
%--------------------------------------------------------------------------
function [grid] = update_brush_fire_grid(grid, height_m, width_m, res, y_list, x_list, values)
% dimensions of brush fire grid
[height_cell, width_cell] = size(grid);

% codebook of y and x values
code_y = linspace(0, height_m, height_m * res);
code_x = linspace(0, width_m, width_m * res);

% partition of y and x values
part_y = code_y(1:end-1) + diff(code_y(1:end)) / 2;
part_x = code_x(1:end-1) + diff(code_x(1:end)) / 2;

% update grid values
for ii=1:size(x_list, 1)
    % determine indices associated with position
    y_ind = quantiz(y_list(ii), part_y) + 1;
    x_ind = quantiz(x_list(ii), part_x) + 1;
    lin_ind = sub2ind([height_cell, width_cell], y_ind, x_ind);
    
    % update the brush fire grid value
    grid(lin_ind) = values(ii);
end

% queue used for propagating grid values
propagate_queue = zeros(1e6, 1);
queue_size = 0;
queue_ind = 1;

% keep track of visited cells
cell_visited = false(height_cell, width_cell);

% start with nighbors of desired position
desired_lin = find(grid == 0, 1);
neighbors = get_neighbors(desired_lin, height_cell, width_cell);
num_neighbors = numel(neighbors);
propagate_queue(queue_size+1:queue_size+num_neighbors) = neighbors;
queue_size = queue_size + num_neighbors;

% propagate grid values
while queue_ind <= queue_size
    % current cell in grid
    cur_cell = propagate_queue(queue_ind);
    cur_val = grid(cur_cell);
    
    if ~cell_visited(cur_cell) && ~isinf(cur_val) && cur_val ~= 0
        % not final position or obstacle position
        % determine value of neighboring cells
        neighbors = get_neighbors(cur_cell, height_cell, width_cell);
        neighbor_values = grid(neighbors);
        
        % update the current cell's value
        new_val = 1 + min(neighbor_values);
        grid(cur_cell) = new_val;
        
        % add unvisited neighbors to queue
        visited_neighbors = cell_visited(neighbors);
        neighbors(visited_neighbors) = [];
        num_neighbors = size(neighbors, 1);
        propagate_queue(queue_size + 1 : queue_size + num_neighbors, :) = neighbors;
        queue_size = queue_size + num_neighbors;
        
        % mark cell as visited
        cell_visited(cur_cell) = true;
    end
    
    % move on to next cell for propagating
    queue_ind = queue_ind + 1;
end
end
%--------------------------------------------------------------------------
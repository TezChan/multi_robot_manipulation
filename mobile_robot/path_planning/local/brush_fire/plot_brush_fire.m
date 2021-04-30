%--------------------------------------------------------------------------
% purpose: plot brush fire grid
%  input:   pos_init = initial position
%          pos_final = desired position
%               grid = brush fire grid
%           height_m = height of brush fire grid in meters
%            width_m = width of brush fire grid in meters
%                res = resolution of brush fire grid in cells per meter
%                fig = figure
% output:        fig = figure
%--------------------------------------------------------------------------
function [fig] = plot_brush_fire(pos_init, pos_final, grid, height_m, width_m, res, fig)
% dimensions of brush fire grid
[height_cell, width_cell] = size(grid);

% codebook of y and x values
code_y = linspace(0, height_m, height_m * res);
code_x = linspace(0, width_m, width_m * res);

% partition of y and x values
part_y = code_y(1:end-1) + diff(code_y(1:end)) / 2;
part_x = code_x(1:end-1) + diff(code_x(1:end)) / 2;

% get linear index of intial position
y_ind = quantiz(pos_init(2), part_y) + 1;
x_ind = quantiz(pos_init(1), part_x) + 1;
lin_ind = sub2ind([height_cell, width_cell], y_ind, x_ind);

% brush fire value at initial position
cur_val = grid(lin_ind);

% build path
path = lin_ind;
while cur_val ~= 0
    neighbors = get_neighbors(lin_ind, height_cell, width_cell);
    [~,ii] = min(grid(neighbors));
    lin_ind = neighbors(ii);
    path = [path; lin_ind];
    cur_val = grid(lin_ind);
end

% get x and y subscripts from linear indices
[path_y, path_x] = ind2sub([height_cell, width_cell], path);

% plot brush fire grid
figure(fig);
clf(fig);
pcolor(code_y, code_x, grid);
hold on;
plots(1) = plot(pos_init(1), pos_init(2), 'm*', 'MarkerSize', 10, 'LineWidth', 2);
plots(2) = plot(pos_final(1), pos_final(2), 'mp', 'MarkerSize', 10, 'LineWidth', 2);
plots(3) = plot(code_x(path_x), code_y(path_y), 'r', 'LineWidth', 3);
xlabel('X');
ylabel('Y');
title('brush fire grid');
legend(plots, {'source position', 'sink position', 'desired path'}, 'Location', 'east');
view(-90, 90);
axis([-1 11 -5 11]);
hold off;
end
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
% purpose: get neighboring grid cells
%  input:   lin_ind = linear index of cell
%            height = height of grid in cells
%             width = width of grid in cells
% output: neighbors = linear indices of neighboring cells
%--------------------------------------------------------------------------
function [neighbors] = get_neighbors(lin_ind, height, width)
[y_ind, x_ind] = ind2sub([height, width], lin_ind);
neighbors = [y_ind, x_ind - 1; y_ind, x_ind + 1; y_ind - 1, x_ind; y_ind + 1, x_ind];
valid_neighbors = neighbors(:,1) > 0 & neighbors(:,1) <= height & neighbors(:,2) > 0 & neighbors(:,2) <= width;
neighbors = neighbors(valid_neighbors, :);
neighbors = sub2ind([height, width], neighbors(:,1), neighbors(:,2));
end
%--------------------------------------------------------------------------
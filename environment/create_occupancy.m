%--------------------------------------------------------------------------
% purpose: create 2D occupancy map of 3D environment
%  input:   width = occupancy map width in meters
%          height = occupancy map height in meters
%             res = occupancy map resolution in cells per meter
%               N = number of points in x and y range
%          colobj = cell array of collision boxes for environment
% output: occ_map = 2D binary occupancy map
%--------------------------------------------------------------------------
function [occ_map] = create_occupancy(width, height, res, N, colobj)
% create map
occ_map = binaryOccupancyMap(width, height, res);

for ii=1:numel(colobj)
    % create circle
    if isa(colobj{ii}, 'collisionCylinder')
        % create a new map for this object
        new_map = binaryOccupancyMap(width, height, res);
        
        % set occupancy of points
        setOccupancy(new_map, colobj{ii}.Pose(1:2,4)', 1);
        
        % inflate points to be circle with desired radius
        inflate(new_map, colobj{ii}.Radius);
        
        % occupied grid indices after inflation
        [occ_row, occ_col] = find(getOccupancy(new_map));
        
        % update full map
        setOccupancy(occ_map, [occ_row, occ_col], 1, 'grid');
    else
        % create rectangle
        rect_cent = colobj{ii}.Pose(1:2,4);
        rect_x = linspace(-colobj{ii}.X / 2, colobj{ii}.X / 2, N);
        rect_y = linspace(-colobj{ii}.Y / 2, colobj{ii}.Y / 2, N);
        [rect_x, rect_y] = meshgrid(rect_x, rect_y);
        new_pts = [rect_x(:), rect_y(:)];
        [~, ang] = R2kth(colobj{ii}.Pose(1:3,1:3));
        new_pts = new_pts * [cos(ang) -sin(ang); sin(ang) cos(ang)]' + rect_cent';
        
        % set occupancy of points
        setOccupancy(occ_map, new_pts, 1);
    end
end
end
%--------------------------------------------------------------------------
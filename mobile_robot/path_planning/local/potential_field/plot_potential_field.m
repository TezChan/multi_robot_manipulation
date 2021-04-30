%--------------------------------------------------------------------------
% purpose: plot potential field
%  input:       pos_final = desired position
%              free_space = collection of valid 2D positions for the robot
%              map_points = 2D map points from scanner measurements
%                gain_att = attractive potential gain
%                gain_rep = repulsive potential gain
%              col_buffer = distance to object when repulsive potential
%                           kicks in
%         distance_thresh = threshold distance when switching from conic
%                           attractive potential to quadratic attractive
%                           potential
%                     fig = figure
% output:             fig = figure
%--------------------------------------------------------------------------
function [fig] = plot_potential_field(pos_final, free_space, map_points,...
    gain_att, gain_rep, col_buffer, distance_thresh, fig)
% gradient at each free space configuration
grad = zeros(size(free_space));

% calculate gradient at each free space location
for ii=1:size(free_space, 1)
    % 2D robot position
    pos_robot = free_space(ii,:)';

    % distance from robot to goal position
    distance = norm(pos_robot - pos_final);

    if distance > distance_thresh
        % conic attractive potential for when robot is far from goal
        grad_u_att = distance_thresh * gain_att * (pos_robot - pos_final) / distance;
    else
        % quadratic attractive potential for when robot is close to goal
        grad_u_att = gain_att * (pos_robot - pos_final);
    end

    % default repulsive gradient
    grad_u_rep = [0; 0];

    if ~isempty(map_points)
        % 2D distances between robot and mapped obstacle points
        deltas = pos_robot - map_points';
        distances = vecnorm(deltas);
        in_range = distances <= col_buffer;
        deltas(:,~in_range) = [];
        distances(~in_range) = [];

        % repulsive gradient
        if any(in_range)
            grad_u_rep = sum(gain_rep * (1 / col_buffer - 1 ./ distances) ./ distances .^2 .* deltas, 2);
        end
    end

    % total gradient
    grad_u = grad_u_att + grad_u_rep;
    grad(ii,:) = -grad_u / norm(grad_u);
end

figure(fig);
clf(fig);
quiver(free_space(:,1), free_space(:,2), grad(:,1), grad(:,2));
hold on;
plot_ind = 1;
plots(plot_ind) = plot(pos_final(1), pos_final(2), 'mp', 'MarkerSize', 10, 'LineWidth', 2);
plot_labels{plot_ind} = 'sink position';
xlabel('X');
ylabel('Y');
title('potential field');
legend(plots, plot_labels, 'Location', 'east');
view(-90,90);
axis([-1 11 -1 11]);
hold off;
end
%--------------------------------------------------------------------------
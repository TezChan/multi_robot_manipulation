%--------------------------------------------------------------------------
% purpose: use potential algorithm for path planning
%  input:           x_est = estimated state
%               pos_final = desired position
%              map_points = 2D map points from scanner measurements
%              max_linear = maximum linear velocity
%             max_angular = maximum angular velocity
%                gain_att = attractive potential gain
%                gain_rep = repulsive potential gain
%              col_buffer = distance to object when repulsive potential
%                           kicks in
%         distance_thresh = threshold distance when switching from conic
%                           attractive potential to quadratic attractive
%                           potential
%                   alpha = step size of gradient descent
% output:               u = control input
%--------------------------------------------------------------------------
function [u] = potential_field(x_est, pos_final, map_points, max_linear, max_angular,...
    gain_att, gain_rep, col_buffer, distance_thresh, alpha)
% 2D robot position
pos_robot = x_est(1:2);

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
grad = grad_u_att + grad_u_rep;

% perturbation to address local minima
grad = grad + 0.5 * randn(2,1);

% new position
pos_new = pos_robot - alpha * grad;
delta_pos = pos_new - pos_robot;
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
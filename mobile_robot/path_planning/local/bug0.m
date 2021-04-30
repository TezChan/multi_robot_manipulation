%--------------------------------------------------------------------------
% purpose: use bug0 algorithm for path planning
%  input:        x_est = estimated state
%            pos_final = desired position
%              z_scanner = scanner measurements
%            ang_scanner = scanner angles
%           col_buffer = closest distance allowed to obstacle to prevent
%                        collision
%           max_linear = maximum linear velocity
%          max_angular = maximum angular velocity
% output:            u = control input
%--------------------------------------------------------------------------
function [u] = bug0(x_est, pos_final, z_scanner, ang_scanner, col_buffer, max_linear, max_angular)
% determine heading to desired position
delta_pos = pos_final - x_est(1:2);
ang_desired = mod(atan2(delta_pos(2), delta_pos(1)) - x_est(3), 2*pi);

% find scanner angle closest to desired angle
[dist_desired, ind_desired] = min(abs(ang_desired - ang_scanner));

% forward facing scanner angles if facing in desired heading
fwd_desired = mod(abs(ang_scanner - ang_scanner(ind_desired)), 2*pi) < pi/2;

% distance to obstacle in desired heading
dist_obstacle = min(z_scanner(fwd_desired));

if dist_obstacle < col_buffer
    % too close to obstacle to travel in desired direction
    % find closest point of contact
    [~, ind_closest] = min(z_scanner);
    
    % desired heading is now tangential to closest point of contact
    ang_desired = mod(ang_scanner(ind_closest) + pi / 2, 2*pi);
    
    % move forward
    dist_desired = max_linear;
end

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

% total commands
u = [u_ang, u_lin];
end
%--------------------------------------------------------------------------
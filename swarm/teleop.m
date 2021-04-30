%--------------------------------------------------------------------------
% purpose: use keyboard teleoperation to control swarm
%  input: swarm = swarm object
%           key = key pressed by user
% output: swarm = swarm object
%--------------------------------------------------------------------------
function [swarm] = teleop(swarm, key)
a = [0; 0];
b = 0;
c = 0;
s = 0.5;

switch key
    case 'w'
        a(1) = s;
    case 's'
        a(1) = -s;
    case 'a'
        a(2) = s;
    case 'd'
        a(2) = -s;
    case 'z'
        b = s;
    case 'x'
        b = -s;
    case 'e'
        c = s/3;
    case 'r'
        c = -s/3;
end

% update swarm position
swarm_position = swarm.position + a;
swarm = set_position(swarm, swarm_position);

% update swarm angle
swarm_angle = swarm.angle + b;
swarm = set_angle(swarm, swarm_angle);

% update swarm radius
swarm_radius = max(swarm.radius + c, 0.25);
swarm = set_radius(swarm, swarm_radius);
end
%--------------------------------------------------------------------------
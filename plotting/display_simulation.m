%--------------------------------------------------------------------------
% purpose: display simulation
%   input:    robot = mobile robot object
%            colobj = cell array of collision boxes for environment
%           fig_num = number of figure
%  output:      sim = figure handle for simulation
%--------------------------------------------------------------------------
function [sim] = display_simulation(robot, colobj, fig_num)
sim = figure(fig_num);
clf(sim);

% display walls
col = rand(1,3);
[~,patchObj] = show(colobj{1});
patchObj.FaceColor = col;
patchObj.EdgeColor = 'none';
hold on;
for ii=2:4
    [~, patchObj] = show(colobj{ii});
    patchObj.FaceColor = col;
    patchObj.EdgeColor = 'none';
end

% display desks
[~,patchObj] = show(colobj{5});
patchObj.FaceColor = rand(1,3);
patchObj.EdgeColor = 'none';
[~,patchObj] = show(colobj{6});
patchObj.FaceColor = rand(1,3);
patchObj.EdgeColor = 'none';

% display stools
[~,patchObj] = show(colobj{7});
patchObj.FaceColor = rand(1,3);
patchObj.EdgeColor = 'none';
[~,patchObj] = show(colobj{8});
patchObj.FaceColor = rand(1,3);
patchObj.EdgeColor = 'none';
[~,patchObj] = show(colobj{9});
patchObj.FaceColor = rand(1,3);
patchObj.EdgeColor = 'none';

% display other obstacles
[~,patchObj] = show(colobj{10});
patchObj.FaceColor = rand(1,3);
patchObj.EdgeColor = 'none';
[~,patchObj] = show(colobj{11});
patchObj.FaceColor = rand(1,3);
patchObj.EdgeColor = 'none';
[~,patchObj] = show(colobj{12});
patchObj.FaceColor = rand(1,3);
patchObj.EdgeColor = 'none';
[~,patchObj] = show(colobj{13});
patchObj.FaceColor = rand(1,3);
patchObj.EdgeColor = 'none';
[~,patchObj] = show(colobj{14});
patchObj.FaceColor = rand(1,3);
patchObj.EdgeColor = 'none';

% display robot
[~,patchObj] = show(robot.col_body);
patchObj.FaceColor = [1 0 0];
patchObj.EdgeColor = 'none';

sim.CurrentAxes.Children(end).Visible = 'off';

view(-90,90);
axis([-1 11 -1 11 0 4]);

rotate3d off;
zoom off;
brush off;
datacursormode off;
pan off;
end
%--------------------------------------------------------------------------
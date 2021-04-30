%--------------------------------------------------------------------------
% purpose: setup environment collision boxes
%   input:
%  output: colobj = cell array of collision boxes for environment
%--------------------------------------------------------------------------
function [colobj] = setup_environment()
ez=[0;0;1];

% walls
wL=10;wW=.5;wH=4;
colobj{1}=collisionBox(wL,wW,wH);
colobj{1}.Pose(1:3,4)=[wL/2;-wW/2;wH/2];
colobj{2}=collisionBox(wL,wW,wH);
colobj{2}.Pose(1:3,4)=[wL/2;wL+wW/2;wH/2];
colobj{3}=collisionBox(wL,wW,wH);
colobj{3}.Pose(1:3,1:3)=rot(ez,pi/2);
colobj{3}.Pose(1:3,4)=[-wW/2;wL/2;wH/2];
colobj{4}=collisionBox(wL,wW,wH);
colobj{4}.Pose(1:3,1:3)=rot(ez,pi/2);
colobj{4}.Pose(1:3,4)=[wL+wW/2;wL/2;wH/2];

% desks
dL=1;dW=2;dH=.9;
desk1loc=[5;5];desk2loc=[2;8];
colobj{5}=collisionBox(dL,dW,dH);
colobj{6}=collisionBox(dL,dW,dH);
colobj{5}.Pose(1:3,4)=[desk1loc;dH/2];
colobj{6}.Pose(1:3,4)=[desk2loc;dH/2];
colobj{6}.Pose(1:3,1:3)=rot(ez,pi/2);

% stools
sR=.25;sL=.4;
colobj{7}=collisionCylinder(sR,sL);
colobj{8}=collisionCylinder(sR,sL);
colobj{9}=collisionCylinder(sR,sL);
colobj{7}.Pose(1:3,4)=[4.2;4.2;sL/2];
colobj{8}.Pose(1:3,4)=[4.2;5.8;sL/2];
colobj{9}.Pose(1:3,4)=[2;7;sL/2];

% other obstacles
colobj{10}=collisionBox(1,2,2);
colobj{11}=collisionBox(1,2,2);
colobj{12}=collisionBox(2,1,2);
colobj{13}=collisionBox(2,1.5,2);
colobj{14}=collisionCylinder(.5,1.5);
colobj{10}.Pose(1:3,4)=[9.5;8;1];
colobj{11}.Pose(1:3,4)=[9.5;1;1];
colobj{12}.Pose(1:3,4)=[4;.5;1];
colobj{13}.Pose(1:3,4)=[6;.5;1];
colobj{14}.Pose(1:3,4)=[8;5;.75];
end
%--------------------------------------------------------------------------
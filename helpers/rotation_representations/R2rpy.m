%
% R2rpy.m
%
% converts R in SO(3) to roll-pitch-yaw
%

function [r,p,y]=R2rpy(R)
  
  r=atan2(R(2,1),R(1,1));
  p=atan2(-R(3,1),sqrt(R(1,1)^2+R(2,1)^2));
  y=atan2(R(3,2),R(3,3));
  
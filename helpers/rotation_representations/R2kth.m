%
% R2kth.m
%
% converts R in SO(3) to (k,theta)
%

function [k,theta]=R2kth(R)
  
  sin_theta=norm(vee(R-R')/2);
  k=vee(R-R')/2/sin_theta;
  theta=atan2(sin_theta,(trace(R)-1)/2);
  
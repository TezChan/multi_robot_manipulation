%
% R2beta.m
%
% converts R in SO(3) to axis-angle product (k theta)
%

function beta=R2beta(R)
  
  beta=vee(logm(R));

  
  
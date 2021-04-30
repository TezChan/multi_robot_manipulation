%
% rpy2R.m
%
% converts roll-pitch-yaw to R in SO(3)
%

function R=rpy2R(r,p,y)
  
  z0=[0;0;1];y0=[0;1;0];x0=[1;0;0];
  R = expm(crossmat(z0)*r)*...
      expm(crossmat(y0)*p)*...
      expm(crossmat(x0)*y);
  
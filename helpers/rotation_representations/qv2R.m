%
% qv2R.m
%
% converts vector quaternion qv to R in SO(3)
%

function R=qv2R(qv)
  
    q0=sqrt(1-norm(qv)^2);
    R=eye(3,3)+2*hat(qv)*(q0*eye(3,3)+hat(qv));  
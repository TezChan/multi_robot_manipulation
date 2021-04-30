%
% q2R.m
%
% converts unit quaternion (q0,qvec) to R in SO(3)
%

function R=q2R(q)
  
  R=eye(3,3)+2*crossmat(q(2:4))*(q(1)*eye(3,3)+crossmat(q(2:4)));
  
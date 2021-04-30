%
% kth2R.m
%
% converts (k,theta) to R in SO(3)
%

function R=kth2R(k,theta)
  
  k=k/norm(k);
  R=eye(3,3)+sin(theta)*crossmat(k)+(1-cos(theta))*crossmat(k)*crossmat(k);
  
%
% beta2R.m
%
% converts axis-angle product to R
%

function R=beta2R(beta)
  
  R=expm(crossmat(beta));
  
  
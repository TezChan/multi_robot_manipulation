%
% R2qv.m
%
% converts R in SO(3) to vector quaternion q_vec
%

function qv=R2qv(R)
  
  q01=.5*sqrt(trace(R)+1);
  q02=-.5*sqrt(trace(R)+1);
  if abs(q01)<1e-5
    [k,theta]=R2kth(R);
    qv=k;
  else
    qv(:,1)=vee(R-R')/4/q01;
    qv(:,2)=vee(R-R')/4/q02;
  end
  
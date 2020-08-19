function [conv Err]=ErrSol(qs1,qs0,epsilon)
% Check is solution is converging
%   conv      : logical variable for  convergence
%               Err<epsilon for each element
%   Err       : Difference between qs1 and qs0 
%   qs1       : new solution
%   qs0       : old solution
%   epsilon   : precision for each variable
nbele=length(qs0);
Err=(qs1-qs0);
conv=1;
for i=1:nbele
  conv=conv & abs(Err(i)) < epsilon;
end

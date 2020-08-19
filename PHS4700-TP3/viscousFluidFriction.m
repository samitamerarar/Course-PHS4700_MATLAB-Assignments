function [ Fv ] = viscousFluidFriction(A,v)
%VISCOUSFLUIDFRICTION Calculate effect on a motion of a ball
%           in a Viscous Fluid (drag).
%
% Syntax:  [ Fv ] = viscousFluidFriction(r,v)
%
% -Inputs-
%        a      :   air of the ball or can in meters
%        v      :   Velocity of the ball or can
%
% -Outputs-
%        Fv     :   Drag
%
% Constants

k = 0.1; %kg/(m2s)

Fv = -A*k*v;
end

function [ vect ] = movementInterval_Can(q,t)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

m = 0.075;
R = 0.15/sqrt(6);
h=0.15;
A = R^2+h^2;

ri = q(1:3);
vi = q(4:6);
teti = q(7:9);
wi = q(10:12);
Fg = gravityForce(m);
Fv = viscousFluidFriction(A,vi);
F = Fg+Fv;

a = F/m;
% Calculate velocity
vv = vi+a*t;
% Calculate  position
rv = ri+vi*t;
tet = teti+wi*t;

vect = [rv-ri,vv-vi,tet-teti,0,0,0];

end


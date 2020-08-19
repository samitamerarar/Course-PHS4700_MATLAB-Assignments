function [ vect ] = kutta_Ball(q,t)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

m = 0.075;
R = 0.0335;
A = R^2*pi;

ri = q(1:3);
vi = q(4:6);
Fg = gravityForce(m);
Fv = viscousFluidFriction(A,vi);
F = Fg+Fv;

a = F/m;
% Calculate velocity
vv = vi+a*t;
% Calculate  position
rv = ri+vi*t;


vect = [rv-ri,vv-vi,0,0,0,0,0,0];

end


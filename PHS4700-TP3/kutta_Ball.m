function [ vect ] = kutta_Ball(q,t)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
ri = q(1:3);
vi = q(4:6);
% Calculate  position
rv = ri+vi*t;


vect = [rv-ri,q(4:12)];

end


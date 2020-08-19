function [ Fg ] = gravityForce(m)
%GRAVITYFORCE Calculate gravity effect on a given ball. (9.8 m/s^2)
%
% Syntax:  [ Fg ] = gravityForce(m)
%
% -Inputs-
%        m      :   Mass of the ball in kg
%
% -Outputs-
%        Fg     :   Gravity Force
%

g = 9.8; % Gravity acceleration
Fg = m * g * [0 0 -1];
end
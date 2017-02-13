function [ F ] = rotor_lift( rpm )
%ROTOR_LIFT Summary of this function goes here
%   Detailed explanation goes here

F = sum(lift(rpm, .30, .12, 1));

end


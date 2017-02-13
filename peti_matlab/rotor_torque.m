function [ T ] = rotor_torque( rpm, arm )
%ROTOR_TORQUE Computes torque produced by rotors.
%   power - rpm of 4 rotors
%           >   y   <
%           1       2
%             \ ^ /
%               |       x
%             /   \
%           3       4
%           >       <
%   arm - length of the quadcopter arm, meters

lifts = lift(rpm, .30, .12, 1);
drags = drag(rpm, .30, .12, 1);
Tx = arm / sqrt(2) * ((lifts(1) + lifts(2)) - (lifts(3) + lifts(4)));
Ty = arm / sqrt(2) * ((lifts(1) + lifts(3)) - (lifts(2) + lifts(4)));
Tz = drags(1) - drags(2) + drags(4) - drags(3);

T = [Tx, Ty, Tz]';

end

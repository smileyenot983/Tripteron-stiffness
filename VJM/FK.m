function [T] = FK(T_base,T_tool,q,t,l_1,l_2,leg)
%FK_1 Summary of this function goes here
% Calculating forward kinematics for given leg

%   Detailed explanation goes here
% INPUTS:
% T_base - transformation from global frame to local frame of leg
% T_tool - transformation from last joint to end effector
% q - transformations for each joint [translation,rotation,rotation,rotation] to reach end effector position
% t - thetas(all zero)
% l_1,l_2 link lengths(equal for each leg)
% leg - leg of robot tripteron 'x' or 'y' or 'z'

% OUTPUTS:
% transformation matrix


%     here it's just multiplication of homogeneous matricesm, each of them has size4x4
if leg=='x'
    T = T_base * Tx(q(1)) * Tx(t(1)) * Rx(q(2)) * Ty(l_1) * Rx(t(2)) * Ry(t(3)) * Rz(t(4)) * Tx(t(5)) * Ty(t(6)) * Tz(t(7)) ...
    *Rx(q(3)) * Ty(l_2) * Rx(t(8)) * Ry(t(9)) * Rz(t(10)) * Tx(t(11)) * Ty(t(12)) * Tz(t(13)) * Rx(q(4)) * T_tool;
end

if leg=='y'
    T = T_base * Ty(q(1)) * Ty(t(1)) * Ry(q(2)) * Tz(l_1) * Rx(t(2)) * Ry(t(3)) * Rz(t(4)) * Tx(t(5)) * Ty(t(6)) * Tz(t(7)) ...
    *Ry(q(3)) * Tz((l_2)) * Rx(t(8)) * Ry(t(9)) * Rz(t(10)) * Tx(t(11)) * Ty(t(12)) * Tz(t(13)) * Ry(q(4)) *T_tool;
end

if leg =='z'
    T = T_base * Tz(q(1)) * Tz(t(1)) * Rz(q(2)) * Tx(l_1) * Rx(t(2)) * Ry(t(3)) * Rz(t(4)) * Tx(t(5)) * Ty(t(6)) * Tz(t(7)) ...
    *Rz(q(3)) * Tx(l_2) * Rx(t(8)) * Ry(t(9)) * Rz(t(10)) * Tx(t(11)) * Ty(t(12)) * Tz(t(13)) * Rz(q(4)) * T_tool;
end


end


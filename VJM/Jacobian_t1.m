function [Jt] = Jacobian_t1(T,T_base,T_tool,q,t,l_1,l_2)
%JACOBIAN Summary of this function goes here
% Computing jacobian matrix(partial derivatives of FK for each axis w.r.t. parameters theta) 
% Jacobians are computed numerically 


%   Detailed explanation goes here
% INPUTS:
% T_base - transformation from global frame to local frame of leg
% T_tool - transformation from last joint to end effector
% q - transformations for each joint [translation,rotation,rotation,rotation] to reach end effector position
% t - thetas(all zero)
% l_1,l_2 link lengths(equal for each leg)

% OUTPUTS:
% Jacobian matrix with size = 6 x number_of_parameters for leg 'x'


T_0 = T; 
T_0(1:3,4) = [0;0;0];
T_0 = T_0';

% theta1
Td = T_base * Tx(q(1)) * Txd(t(1)) * Rx(q(2)) * Tz((l_1)) * Rx(t(2)) * Ry(t(3)) * Rz(t(4)) * Tx(t(5)) * Ty(t(6)) * Tz(t(7)) ...
    *Rx(q(3)) * Ty((l_2)) * Rx(t(8)) * Ry(t(9)) * Rz(t(10)) * Tx(t(11)) * Ty(t(12)) * Tz(t(13))* Rx(q(4)) * T_tool * T_0;

Jt_1 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;

% theta2
Td = T_base * Txd(q(1)) * Tx(t(1)) * Rx(q(2)) * Tz((l_1)) * Rxd(t(2)) * Ry(t(3)) * Rz(t(4)) * Tx(t(5)) * Ty(t(6)) * Tz(t(7)) ...
    *Rx(q(3)) * Ty((l_2)) * Rx(t(8)) * Ry(t(9)) * Rz(t(10)) * Tx(t(11)) * Ty(t(12)) * Tz(t(13))* Rx(q(4))  * T_tool * T_0;

Jt_2 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;

% theta3
Td = T_base * Tx(q(1)) * Tx(t(1)) * Rx(q(2)) * Tz((l_1)) * Rx(t(2)) * Ryd(t(3)) * Rz(t(4)) * Tx(t(5)) * Ty(t(6)) * Tz(t(7)) ...
    *Rx(q(3)) * Ty((l_2)) * Rx(t(8)) * Ry(t(9)) * Rz(t(10)) * Tx(t(11)) * Ty(t(12)) * Tz(t(13))* Rx(q(4))  * T_tool * T_0;

Jt_3 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;

% theta4
Td = T_base * Tx(q(1)) * Tx(t(1)) * Rx(q(2)) * Tz((l_1)) * Rx(t(2)) * Ry(t(3)) * Rzd(t(4)) * Tx(t(5)) * Ty(t(6)) * Tz(t(7)) ...
    *Rx(q(3)) * Ty((l_2)) * Rx(t(8)) * Ry(t(9)) * Rz(t(10)) * Tx(t(11)) * Ty(t(12)) * Tz(t(13))* Rx(q(4))  * T_tool * T_0;

Jt_4 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;


% theta5
Td = T_base * Tx(q(1)) * Tx(t(1)) * Rx(q(2)) * Tz((l_1)) * Rx(t(2)) * Ry(t(3)) * Rz(t(4)) * Txd(t(5)) * Ty(t(6)) * Tz(t(7)) ...
    *Rx(q(3)) * Ty((l_2)) * Rx(t(8)) * Ry(t(9)) * Rz(t(10)) * Tx(t(11)) * Ty(t(12)) * Tz(t(13))* Rx(q(4))  * T_tool * T_0;

Jt_5 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;

% theta6
Td = T_base * Tx(q(1)) * Tx(t(1)) * Rx(q(2)) * Tz((l_1)) * Rx(t(2)) * Ry(t(3)) * Rz(t(4)) * Tx(t(5)) * Tyd(t(6)) * Tz(t(7)) ...
    *Rx(q(3)) * Ty((l_2)) * Rx(t(8)) * Ry(t(9)) * Rz(t(10)) * Tx(t(11)) * Ty(t(12)) * Tz(t(13))* Rx(q(4))  * T_tool * T_0;

Jt_6 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;

% theta7
Td = T_base * Tx(q(1)) * Tx(t(1)) * Rx(q(2)) * Tz((l_1)) * Rx(t(2)) * Ry(t(3)) * Rz(t(4)) * Tx(t(5)) * Ty(t(6)) * Tzd(t(7)) ...
    *Rx(q(3)) * Ty((l_2)) * Rx(t(8)) * Ry(t(9)) * Rz(t(10)) * Tx(t(11)) * Ty(t(12)) * Tz(t(13)) * Rx(q(4)) * T_tool * T_0;

Jt_7 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;

% theta8
Td = T_base * Tx(q(1)) * Tx(t(1)) * Rx(q(2)) * Tz((l_1)) * Rx(t(2)) * Ry(t(3)) * Rz(t(4)) * Tx(t(5)) * Ty(t(6)) * Tz(t(7)) ...
    *Rx(q(3)) * Ty((l_2)) * Rxd(t(8)) * Ry(t(9)) * Rz(t(10)) * Tx(t(11)) * Ty(t(12)) * Tz(t(13))* Rx(q(4)) * T_tool * T_0;

Jt_8 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;

% theta9
Td = T_base * Tx(q(1)) * Tx(t(1)) * Rx(q(2)) * Tz((l_1)) * Rx(t(2)) * Ry(t(3)) * Rz(t(4)) * Tx(t(5)) * Ty(t(6)) * Tz(t(7)) ...
    *Rx(q(3)) * Ty((l_2)) * Rx(t(8)) * Ryd(t(9)) * Rz(t(10)) * Tx(t(11)) * Ty(t(12)) * Tz(t(13))* Rx(q(4)) * T_tool * T_0;

Jt_9 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;

% theta10
Td = T_base * Txd(q(1)) * Tx(t(1)) * Rx(q(2)) * Tz((l_1)) * Rx(t(2)) * Ry(t(3)) * Rz(t(4)) * Tx(t(5)) * Ty(t(6)) * Tz(t(7)) ...
    *Rx(q(3)) * Ty((l_2)) * Rx(t(8)) * Ry(t(9)) * Rzd(t(10)) * Tx(t(11)) * Ty(t(12)) * Tz(t(13))* Rx(q(4))  * T_tool * T_0;

Jt_10 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;

% theta11
Td = T_base * Tx(q(1)) * Tx(t(1)) * Rx(q(2)) * Tz((l_1)) * Rx(t(2)) * Ry(t(3)) * Rz(t(4)) * Tx(t(5)) * Ty(t(6)) * Tz(t(7)) ...
    *Rx(q(3)) * Ty((l_2)) * Rx(t(8)) * Ry(t(9)) * Rz(t(10)) * Txd(t(11)) * Ty(t(12)) * Tz(t(13))* Rx(q(4)) *  T_tool * T_0;

Jt_11 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;

% theta12
Td = T_base * Tx(q(1)) * Tx(t(1)) * Rx(q(2)) * Tz((l_1)) * Rx(t(2)) * Ry(t(3)) * Rz(t(4)) * Tx(t(5)) * Ty(t(6)) * Tz(t(7)) ...
    *Rx(q(3)) * Ty((l_2)) * Rx(t(8)) * Ry(t(9)) * Rz(t(10)) * Tx(t(11)) * Tyd(t(12)) * Tz(t(13)) * Rx(q(4))*  T_tool * T_0;

Jt_12 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;

% theta13
Td = T_base * Tx(q(1)) * Tx(t(1)) * Rx(q(2)) * Tz((l_1)) * Rx(t(2)) * Ry(t(3)) * Rz(t(4)) * Tx(t(5)) * Ty(t(6)) * Tz(t(7)) ...
    *Rx(q(3)) * Ty((l_2)) * Rx(t(8)) * Ry(t(9)) * Rz(t(10)) * Tx(t(11)) * Ty(t(12)) * Tzd(t(13))* Rx(q(4))  * T_tool * T_0;

Jt_13 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;


Jt = [Jt_1, Jt_2, Jt_3, Jt_4,Jt_5, Jt_6, Jt_7, Jt_8,Jt_9, Jt_10, Jt_11, Jt_12,Jt_13];

end


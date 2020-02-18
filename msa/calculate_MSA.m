function [Kc] = calculate_MSA(leg,end_effector)
%CALCULATE_MSA Summary of this function goes here
% Calculating stiffness matrix 

%   Detailed explanation goes here
% INPUTS:
% leg - chosen leg of tripteron 'x' or 'y' or 'z'
% end efffector - end effector position [x,y,z,angle_x,angle_y,angle_z]

% matrix for rigid base
rigid_base_W = [zeros(6)]; %part which will be multiplied by W
rigid_base_t = [eye(6)]; %part which will be multiplied by delta t


% MSA for ELASTIC JOINT #1(W - part which will be multiplied by W, t- part which will be multiplied by delta t)
[elastic_joint_1_W,elastic_joint_1_t] = elastic_joint(leg);


% MSA for PASSIVE JOINT #2(W - part which will be multiplied by W, t- part which will be multiplied by delta t)              
[passive_joint_2_W,passive_joint_2_t] = passive_joint(leg);

% MSA for PASSIVE JOINT #3(W - part which will be multiplied by W, t- part which will be multiplied by delta t)

[passive_joint_3_W,passive_joint_3_t] = passive_joint(leg);
                 
% MSA for PASSIVE JOINT #4(W - part which will be multiplied by W, t- part which will be multiplied by delta t)

[passive_joint_4_W,passive_joint_4_t] = passive_joint(leg);             

                 

% parameters of elastic links
l_1 = [0.35,10e-4,10e-4];
l_2 = [0.35,10e-4,10e-4];

% Inverse kinematics to obtain positions for each joint
q_x = Inverse(end_effector,l_1(1),l_2(1),leg);

%MSA for ELASTIC LINK #1 (W - part which will be multiplied by W, t- part which will be multiplied by delta t)
[elastic_link_1_W,elastic_link_1_t] = elastic_link(l_1,q_x(2),leg);

%MSA for ELASTIC LINK #2 (W - part which will be multiplied by W, t- part which will be multiplied by delta t)
[elastic_link_2_W,elastic_link_2_t] = elastic_link(l_2,q_x(2),leg);


% here comes the aggregation
      
Aggregated = zeros(84,84);

% here the part with W
% Aggregated part for multiplying with W
Aggregated(1:6,1:6) = rigid_base_W;
Aggregated(7:18,1:12) = elastic_joint_1_W;
Aggregated(19:30,7:18) = passive_joint_2_W;
Aggregated(31:42,13:24) = elastic_link_1_W;
Aggregated(43:54,19:30) = passive_joint_3_W;
Aggregated(55:66,25:36) = elastic_link_2_W;
Aggregated(67:78,31:42) = passive_joint_4_W;
Aggregated(79:84,37:42) = eye(6); %adding identity matrix as it was added in paper from Skvorzova,Popov

%  Aggregated part for multiplying with delta t
Aggregated(1:6,43:48) = rigid_base_t;
Aggregated(7:18,43:54) = elastic_joint_1_t;
Aggregated(19:30,49:60) = passive_joint_2_t;
Aggregated(31:42,55:66) = elastic_link_1_t;
Aggregated(43:54,61:72) = passive_joint_3_t;
Aggregated(55:66,67:78) = elastic_link_2_t;
Aggregated(67:78,73:84) = passive_joint_4_t;

% now i should separate this big matrix into A,B,C,D

A = Aggregated(1:78,1:78);
B = Aggregated(1:78,79:84);
C = Aggregated(79:84,1:78);
D = Aggregated(79:84,79:84);

% obtaining stiffnes matrix as in KLIMCHIK'S presentation
Kc = D - C * inv(A) * B;


end


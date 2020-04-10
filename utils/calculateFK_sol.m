function [jointPositions,T0i] = calculateFK_sol(q, robot)
% CALCULATEFK_SOL This function calculates the forward kinematics for the
%   Lynx manipulator arm with lidar. It is the official solution for the MEAM520
%   assignments.
%
% AUTHOR:
%   Dr. Cynthia Sung (crsung@seas.upenn.edu)
%   Modified by Gedaliah Knizhnik (knizhnik@seas.upenn.edu) 8/28/19
%
% INPUT:
%   q - 1x6 vector of joint inputs [q1,q2,q3,q4,q5,lg]
%
% OUTPUT:
%   jointPositions - 7 x 3 matrix, where each row represents one 
%                    joint along the robot. Each row contains the [x,y,z]
%                    coordinates of the respective joint's center (mm). For
%                    consistency, the first joint should be located at 
%                    [0,0,0]. These values are used to plot the robot.
%   T0i            - 4 x 4 homogeneous transformation matrices
%                    representing {i}th frame expressed in the 
%                    base (0) frame. 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Your code here

% Lynx ADL5 constants in mm
if exist('robot','var')
    d1 = robot.d1; % base height (table to center of joint 2)
    a2 = robot.a2; % shoulder to elbow length
    a3 = robot.a3; %elbow to wrist length
    d4 = robot.d4; %joint 4 location in frame 4
    d5 = robot.d5; %wrist to base of gripper
    d6 = robot.d6; %wrist to lidar
    lg = robot.lg; %length of gripper
else
    % default values
    d1 = 76.2; % base height (table to center of joint 2)
    a2 = 146.05; % shoulder to elbow length
    a3 = 187.325; %elbow to wrist length
    d4 = 34; %joint 4 location in frame 4
    %d5 = 76.2; %wrist to base of gripper
    %lg = 28.575; %length of gripper
    d5 = 52.75;% wrist to lidar
    d6 = 35; % wrist to lidar
end

%Frame 1 w.r.t Frame 0
T1 = [cos(q(1)) -sin(q(1))*cos(-pi/2)  sin(q(1))*sin(-pi/2)  0;
      sin(q(1))  cos(q(1))*cos(-pi/2) -cos(q(1))*sin(-pi/2)  0;
              0            sin(-pi/2)            cos(-pi/2) d1;
              0                     0                  0     1];
          
%Frame 2 w.r.t Frame 1          
T2 = [cos(q(2)-(pi/2)) -sin(q(2)-(pi/2))  0   a2*cos(q(2)-(pi/2));
      sin(q(2)-(pi/2))  cos(q(2)-(pi/2))  0   a2*sin(q(2)-(pi/2));
              0                        0  1                     0;
              0                        0  0                     1];

%Frame 3 w.r.t Frame 2
T3 = [cos(q(3)+(pi/2)) -sin(q(3)+(pi/2))  0   a3*cos(q(3)+(pi/2));
      sin(q(3)+(pi/2))  cos(q(3)+(pi/2))  0   a3*sin(q(3)+(pi/2));
              0                        0  1                     0;
              0                        0  0                     1];

%Frame 4 w.r.t Frame 3
T4 = [cos(q(4)-(pi/2)) -sin(q(4)-(pi/2))*cos(-pi/2)   sin(q(4)-(pi/2))*sin(-pi/2)   0;
      sin(q(4)-(pi/2))  cos(q(4)-(pi/2))*cos(-pi/2)  -cos(q(4)-(pi/2))*sin(-pi/2)   0;
              0                          sin(-pi/2)                    cos(-pi/2)   0;
              0                                   0                             0   1];
%Frame 5 w.r.t Frame 4 
T5 = [-sin(q(5))  0  cos(q(5))  0;
       cos(q(5))  0  sin(q(5))  0;
              0   1     0       d5;
              0   0     0       1];
          
%Frame 5 w.r.t Frame 4 
T6 = [ cos(q(6))  -sin(q(6))  0   0;
       sin(q(6))   cos(q(6))  0   0;
           0            0     1  d6;
           0            0     0   1];

%% Positions of Joints
%Position of First Joint (Base Revolute)
X(1,:) = [0 0 0 1];

%Position of Second Joint (Shoulder Revolute)
X(2,:) = (T1*[0;0;0;1])';

%Position of Third Joint (Elbow Revolute)
X(3,:) = (T1*T2*[0;0;0;1])';

%Position of Fourth Joint (1st Wrist)
X(4,:) = (T1*T2*T3*[0;0;0;1])';

%Position of Fifth Joint (2nd Wrist)
X(5,:) = (T1*T2*T3*T4*[0;0;d4;1])';

%Position of Gripper (Base of the Gripper)
%X(6,:) = (T1*T2*T3*T4*T5*[0;0;0;1])';

%Position of the Sixth Joint (lidar mount)
X(6,:) = (T1*T2*T3*T4*T5*[0;0;0;1]);

%Position of the lidar
X(7,:) = (T1*T2*T3*T4*T5*T6*[0;0;0;1]);

%Outputs the 7x3 of the locations of each joint in the Base Frame
jointPositions = X(:,1:3);


%% Transformation Matrices for Intermediate Frames
T0i = eye(4);
T0i(:,:,2) = T1;
T0i(:,:,3) = T1*T2;
T0i(:,:,4) = T1*T2*T3;
T0i(:,:,5) = T1*T2*T3*T4;
T0i(:,:,6) = T1*T2*T3*T4*T5;
T0i(:,:,7) = T1*T2*T3*T4*T5*T6;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
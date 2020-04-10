function [] = lynxServoPhysical(th1, th2, th3, th4, th5, grip)
% Commands the real Lynx robot to the angles defined by the input (in radians and mm)
% Has the same limits as lynxServo.m.

% INPUTS:
    % th1...th5 : Joint variables for the six DOF Lynx arm (rad)
    % grip = gripper width (mm)

global lynx
if (nargin == 1 && length(th1) == 6)
    q = th1;
else
    q = [th1 th2 th3 th4 th5 grip]; % Position vector
end
%% Adjusting for out of range positions
% The guide for the controller states that 1000us corresponds 90degrees.
% If so, 636.62 converts from radians to microseconds. (1000/90*180/pi)
maxSpeedCommands = lynx.param.maxOmegas*636.62;

% Conversion from joint angles to servo commands.
% according to the controller manual, <pw> should be a number in [500, 2500], so
% here we map [0, pi] to [0, 2000] and include the 500 offset in the servo offsets
P1 = lynx.param.servoDirection(1) * q(1) / pi * lynx.param.servoScales(1) + lynx.param.servoOffsets(1);
P2 = lynx.param.servoDirection(2) * q(2) / pi * lynx.param.servoScales(2) + lynx.param.servoOffsets(2);
P3 = lynx.param.servoDirection(3) * q(3) / pi * lynx.param.servoScales(3) + lynx.param.servoOffsets(3);
P4 = lynx.param.servoDirection(4) * q(4) / pi * lynx.param.servoScales(4) + lynx.param.servoOffsets(4);
P5 = lynx.param.servoDirection(5) * q(5) / pi * lynx.param.servoScales(5) + lynx.param.servoOffsets(5);
P6 = lynx.param.servoDirection(6) * q(6) / pi * lynx.param.servoScales(6) + lynx.param.servoOffsets(6);

%% Sending commands to lynx

fprintf(lynx.serialPort, '%s\r', ...
    ['#1P' num2str(P1,'%.0f') ' S' num2str(maxSpeedCommands(1),'%.0f') ...
     '#2P' num2str(P2,'%.0f') ' S' num2str(maxSpeedCommands(2),'%.0f') ...
     '#3P' num2str(P3,'%.0f') ' S' num2str(maxSpeedCommands(3),'%.0f') ...
     '#4P' num2str(P4,'%.0f') ' S' num2str(maxSpeedCommands(4),'%.0f') ...
     '#5P' num2str(P5,'%.0f') ' S' num2str(maxSpeedCommands(5),'%.0f') ...
     '#6P' num2str(P6,'%.0f') ' S' num2str(maxSpeedCommands(6),'%.0f')]);

end

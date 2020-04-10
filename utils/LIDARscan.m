function [dist, unit] = LIDARscan()
% Read a frame of LIDAR measurement
%
% INPUTS:
%   th1...th6 : Joint variables for the six DOF Lynx arm (radians and
%   mm)
% OUTPUT:
%   dist : distance from the object closest to the end-effector (LIDAR) to each joint
%
%   unit : 1x3 vector, direction from the end effector to the detected object
%

% Ensure the Lynx has been initialized
if(~evalin('base','exist(''lynx'',''var'')'))
    error('Lynx has not been initialised. Run lynxStart first');
end

global lynx

if ~lynx.hardware_on
    warning('Simulation: Will return NaN values');
    dist = NaN;
    unit = nan(1,3);
    return
end

if (~strcmp(lynx.serialPort.status, 'open'))
    error('LIDARscan Error: serial port closed');
end

% clean up the serial port buffer
while (lynx.serialPort.BytesAvailable > 0)
    fgetl(lynx.serialPort);
end
fprintf(lynx.serialPort, 'lidar');
wait_tick = 0;
while (lynx.serialPort.BytesAvailable <= 0)
    wait_tick = wait_tick + 1;
    if wait_tick > 5000
        dist = 1000;
        unit = [0 0 0];
        disp('time out')
        return
    end
end
retval = fgets(lynx.serialPort);
dist = str2double(char(retval));
% conver to mm
dist = dist * 10;


fprintf('%f\n', dist);
angle = 0;
[jointPositions, T0e] = calculateFK_sol([lynx.q(1:5) angle]);

% object point in end effector frame
p_e = [0, dist, 0, 1]';

% object in world frame
p = T0e(:,:,end) * p_e;
p = p(1:3);

% end effector in world frame
endeffector = jointPositions(end,:)';
unit = p - endeffector;
unit = unit / norm(unit);
unit = reshape(unit,1,[]); % Make unit a row vector
end

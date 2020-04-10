function lynxStart(varargin)
% lynxStart  Loads kinematic data for a Lynx AL5D manipulator and sets
%   variables.
%
%   If using hardware:
%   Starts the Lynx and moves the Lynx to its home position.
%
%   WARNING: Be aware of the Lynx's position before turning on the
%   controller. To prevent damage, remove any objects in the path between
%   the current position and the home position. If the Lynx is likely to
%   slam into the table surface or other immovable object, manually move
%   the Lynx towards the home position before using this command.
%
%     Options         Values {Default Value}
%       'Hardware'      'Legend' | 'Lucky' | 'Lyric' | {'off'} Sets up the code to use the Lynx
%                       hardware.  If hardware is enabled then there
%                       is no visualization.  Default is simulation.
%                       Be careful to read the above warning before
%                       enabling hardware.
%
%       'Port'          string. e.g. 'com3'  -  The com port may change.
%                       Go to Device manager on a PC to find the port.
%                       To use the Lynx with a MAC, see instructions in
%                       lynxInitializeHardware.m
%
%       'Joints'        {'on'} | 'off' Display the joints of the robot
%
%       'Frame'         {'on'} | 'off' Display the end effector frame
%
%       'Shadow'        {'on'} | 'off' Plot the shadow of the robot on the
%                       ground plane
%
%       'Gripper'       'on' | {'off'} Plot the gripper.  Cool, but is it
%                       really helpful?
%
%  Updated for MEAM 520, 2018. May still have a few bugs, so please let us
%  know if you find any bugs.  And better yet, suggest how to fix it =)

global lynx pennkeys
addpath('utils')

%%SET THIS
pennkeys = 'sol';

robot_spec_file = 'utils/robot_specs.json';


file_spec = fopen(robot_spec_file);
jsobj = jsondecode(fscanf(file_spec, '%c', inf));
fclose(file_spec);

lynx.param = jsobj.('Simulation');
lynx.servoTimeStamp = tic;
lynx.lidarTimeStamp = tic;
lynx.firstFrame = true;
lynx.showFrame = true;
lynx.showJoints = true;
lynx.showShadow = true;
lynx.showGripper = false;
lynx.hardware_on = false;

% Home pose
lynx.q = [0,0,0,0,0,0];

% Check property inputs is even (each property must be paired with a value)
if mod(size(varargin,2), 2) == 1
    error('Must be a value for each property set')
end

for j = 1:2:size(varargin,2)
    % Use the Lynx hardware or simulation
    if strcmpi(varargin{1,j}, 'Hardware')
        hardware_config = varargin{1,j+1};


        if strcmpi(hardware_config, 'off')
            lynx.hardware_on = false;

        elseif strcmpi(hardware_config, 'Legend') || ...
               strcmpi(hardware_config, 'Lucky')  || ...
               strcmpi(hardware_config, 'Loopy')  || ...
               strcmpi(hardware_config, 'Lyric')

            lynx.hardware_on = true;
            lynx.param = jsobj.(hardware_config);
        else
            error('Invalid value for Hardware property');
        end


    elseif strcmpi(varargin{1,j}, 'Port')
        lynx.serialPort = varargin{1,j+1};

    elseif strcmpi(varargin{1,j}, 'Joints')
        if strcmpi(varargin{1,j+1}, 'off')
            lynx.showJoints = false;
        elseif strcmpi(varargin{1,j+1}, 'on')
            lynx.showJoints = true;
        else
            error('Invalid value for Joints property');
        end

    elseif strcmpi(varargin{1,j}, 'Frame')
        if strcmpi(varargin{1,j+1}, 'off')
            lynx.showFrame = false;
        elseif strcmpi(varargin{1,j+1}, 'on')
            lynx.showFrame = true;
        else
            error('Invalid value for Frame property');
        end

    elseif strcmpi(varargin{1,j}, 'Shadow')
        if strcmpi(varargin{1,j+1}, 'off')
            lynx.showShadow = false;
        elseif strcmpi(varargin{1,j+1}, 'on')
            lynx.showShadow = true;
        else
            error('Invalid value for Shadow property');
        end

    elseif strcmpi(varargin{1,j}, 'Gripper')
        if strcmpi(varargin{1,j+1}, 'off')
            lynx.showGripper = false;
        elseif strcmpi(varargin{1,j+1}, 'on')
            lynx.showGripper = true;
        else
            error('Invalid value for Gripper property');
        end

    end

end

% Initialize the Lynx if using hardware
if lynx.hardware_on

    % Display warning message
    str = input(['Warning: Be aware of the Lynx''s position before continuing.\n'...
        'Hold the lynx as close to the home position as possible.\n' ...
        'Do you want to continue (y/n)?\n'],'s');
    if isempty(str)
        str = 'n';
    end
    if strcmpi(str, 'y') || strcmpi(str, 'yes')
        %Opens serial communication with the lynx

        %serialPort is a string. e.g. 'com3'
        %The com port may change.  Go to Device manager on a PC to find the port.

        %Close any previous communication.
        delete(instrfindall);

        %Alternatively, on a MAC,
        %ttl = serial('/dev/cu.usbserial-AI0484D4');
        %Note that the portion after the hypen depends on the usb cable.  Type
        %ls /dev/cu.* into the command line to find this name.

        lynx.serialPort = serial(lynx.serialPort, 'BaudRate', 115200, 'Terminator', 'CR');
        fopen(lynx.serialPort);

        if(~strcmp(lynx.serialPort.Status,'open'))
            error('Please connect the Lynx robot via the USB port');
        end

        % send a handshaking frame
        % if lynx.controlBoard == 'botboarduino'
            % fprintf(lynx.serialPort, '%s\r', 'tick')
        % end

    else
        disp('Start cancelled.');
    end

% Initialize the plot if using simulation
else
    plotLynx(lynx.q);
end

% Set global variables in the base workspace
evalin('base', 'global lynx pennkeys')

%Send the robot to a home configuration
lynxServo(lynx.q);

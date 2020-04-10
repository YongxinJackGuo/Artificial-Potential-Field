%% Setup

close all
addpath('utils')
addpath('maps')

%% Simulation Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%       Modify this part       %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

start = [0,0,0,0,0,0];
goal = [1.4,0,0,0,0,1];
mapToUse = 'map2.txt';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%   Do not modify after this   %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Load Map and Robot

load 'robot.mat' robot
map = loadmap(mapToUse);
map.start = start;
map.goal  = goal;

%% Setup the Simulation

% If lynx global doesn't exist in the base workspace start Lynx
if(~evalin('base','exist(''lynx'',''var'')'))        
    startLynx = 1;
else
    global lynx
    
    if ~ishandle(lynx.hLinks)
        startLynx = 1;
    else
        startLynx = 0;
    end
end

% Uncomment the correct line to run in either simulation or hardware
if startLynx
    lynxStart();
    % lynxStart('Hardware','Legend','Port','COM3');
    pause(1);
end

plotmap(map)
% Move the lynx to the start position
lynxServo(start);
pause(1);

%% Run the Simulation

% Initially not done and at start location
isDone = 0;
qCurr = start;

% While not finished, take a step
while ~isDone
    % Calculate the potential field step
   [qNext, isDone] = potentialFieldStep_group37(qCurr, map, robot);
   % Take the step
   lynxServo(qNext);
   qCurr = qNext;
   % Pause for simulation
   pause(0.2);  % You may change this to speed up the sim or slow it down
end

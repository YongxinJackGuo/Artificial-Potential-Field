function [] = lynxServo(th1, th2, th3, th4, th5, th6)
% Commands the Lynx to the angles defined by the input (in radians and mm)
%
% INPUTS:
%   th1...th6 : Joint variables for the six DOF Lynx arm (radians and
%   mm)
% OUTPUT:
%   Calls lynxServoSim.m and displays the Lynx in the configuration specified by
%   the input parameters, or calls lynxServoPhysical.m to move the real
%   robot.

if nargin

    % Ensure the Lynx has been initialized
    if(~evalin('base','exist(''lynx'',''var'')'))
        error('lynxServo:lynxInitialization',...
            'Lynx has not been initialised. Run lynxStart first');
    end

    % Import global variables
    global lynx

    % Check inputs (whether q is 6 entries or single vector)
    if nargin == 6 && length(th1) == 1
        qNew = [th1 th2 th3 th4 th5 th6];
    elseif nargin == 1 && length(th1) == 6 && numel(th1) == 6
        qNew = th1;
        % Ensure that q is a row vector
        if size(qNew, 1) > 1
            qNew = qNew';
        end
    else
        error(['Input must be 6 scalars or a 6 element vector representing', ...
                ' desired joint angles followed by the robot name.'])
    end

    if any(isnan(qNew))
        error('lynxServo:nanInput', ...
            ['Input contains a value of NaN. Input was: [' num2str(qNew) ']']);
    end

    % Check that angular limits are satisfied
    for i=1:length(qNew)
        if qNew(i) < lynx.param.lowerLim(i)
            qNew(i) = lynx.param.lowerLim(i);
            fprintf('Joint %d was sent below lower limit, moved to boundary %0.2f\n', ...
                i,lynx.param.lowerLim(i))
        elseif qNew(i) > lynx.param.upperLim(i)
            qNew(i) = lynx.param.upperLim(i);
            fprintf('Joint %d was sent above upper limit, moved to boundary %0.2f\n', ...
                i,lynx.param.upperLim(i))
        end
    end

    % Check that angular velocity limits are satisfied
    dt = toc(lynx.servoTimeStamp); %time since velocity limit was last checked
    lynx.servoTimeStamp = tic;

    if dt > 0
        omega = abs((qNew - lynx.q)/dt);
        speedind = find(omega>lynx.param.maxOmegas');
        if ~isempty(speedind)
            jointErrStr = [];
            for i = 1:length(speedind)
                jointErrStr = [jointErrStr ...
                    sprintf(['\nJoint %d has approached an angular', ...
                    'velocity of %.3f rad/s. The max angular velocity for ', ...
                    'joint %d is %0.3f rad/s'], ...
                    speedind(i), omega(speedind(i)),...
                    speedind(i),lynx.param.maxOmegas(speedind(i)))];
            end
           error('lynxServo:VelocityLimit', '%s', jointErrStr);
        end
    end

    lynx.q = qNew;

    % Send the angles to the Lynx
    if lynx.hardware_on
        lynxServoPhysical(lynx.q);
    else
        lynxServoSim(lynx.q);
    end

end

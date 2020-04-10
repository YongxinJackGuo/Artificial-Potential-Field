function [ ] = lynxServoSim(th1, th2, th3, th4, th5, grip)
% lynxServoSim moves the simulated Lynx to the new configuration specified by the input (in radians and mm)

% INPUTS:
    % th1...th5 : Joint variables for the six DOF Lynx arm (rad)
    % grip = gripper width (mm)

    % Ensure that the Lynx has been initialized
    if(~evalin('base','exist(''lynx'',''var'')'))
        error('lynxMove:lynxInitialization','Lynx has not been initialised. Run lynxStart first');
    end

    % Check inputs (whether q is 6 entries or single vector)
    if nargin == 6 && length(th1) == 1
        q = [th1 th2 th3 th4 th5 grip];
    elseif nargin == 1 && length(th1) == 6 && numel(th1) == 6
        q = th1;
        % Ensure that q is a row vector
        if size(q, 1) > 1
            q = q';
        end
    else
        error('lynxMove:argumentCheck','Input must be 6 scalars or a 6 element vector representing desired joint angles.')
    end

    if any(isnan(q))
        error('lynxMove:nanInput',['Input contains a value of NaN. Input was: [' num2str(q) ']']);
    end

    % Changes the Lynx config to that specified by input and plots it there
    plotLynx(q);

end


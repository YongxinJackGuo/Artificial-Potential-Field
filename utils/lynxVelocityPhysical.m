function [] = lynxVelocityPhysical(om1, om2, om3, om4, om5, gripV)
% Commands the real Lynx robot to the angular velocities defined by inputs (in rad/s and mm/s)
% Has the same limits as lynxServo.m

% NOTE: The Lynx robots are not very good at this.  If left to its own
% devices, each robot would try to continue moving beyond its physical
% limits. To handle this, we actually send each joint to either its max or
% min position at a given velocity.  Once the joint reaches its max or min,
% it stops.

% INPUTS:
%   om1...om6 : Joint variables for the six DOF Lynx arm (th6 = grip)

qdot = [om1 om2 om3 om4 om5 gripV]; % Position vector

%% Adjusting for out of range positions
% Values below this are set to zero
smallestVels = pi/30*ones(1,6);

for i=1:length(qdot)
    if qdot(i) < -lynx.param.maxOmegas(i)
        qdot(i) = -lynx.param.maxOmegas(i);
        fprintf('Joint %d was sent below velocity limit, changed to -%0.2f\n',i,lynx.param.maxOmegas(i))
    elseif qdot(i) > lynx.param.maxOmegas(i)
        qdot(i) = lynx.param.maxOmegas(i);
        fprintf('Joint %d was sent above velocity limit, changed to %0.2f\n',i,lynx.param.maxOmegas(i))
    end
end

%Calculate max and min positions
for(i = 1:5)
    Pmax(i) = lynx.param.upperLim(i) * (180/pi/lynx.param.ratios(i)) + lynx.param.servoOffsets(i);
    Pmin(i) = lynx.param.lowerLim(i) * (180/pi/lynx.param.ratios(i)) + lynx.param.servoOffsets(i);
end
Pmax(6) = lynx.param.upperLim(6) / -0.028 + lynx.param.servoOffsets(6);
Pmin(6) = lynx.param.lowerLim(6) / -0.028 + lynx.param.servoOffsets(6);

S(1) = qdot(1) * (180/pi/0.102);
S(2) = qdot(2) * (180/pi/0.105);
S(3) = qdot(3) * (180/pi/0.109);
S(4) = qdot(4) * (180/pi/0.095);
S(5) = qdot(5) * (180/pi/0.102);
S(6) = qdot(6) / 0.028;

str{1} = '';
str{2} = '';
str{3} = '';
str{4} = '';
str{5} = '';
str{6} = '';

for(i=1:6)
    if(S(i) > 0) %If velocity is positive
        P(i) = Pmax(i);
    else %Velocity is negative
        P(i) = Pmin(i);
    end
    if( abs(S(i)) > smallestVels(i) )
        str{i} = ['#' num2str(i,'%.0f') 'P' num2str(P(i),'%.0f') ' S' num2str(S(i),'%.0f')];
    else
        str{i} = ['STOP' num2str(i,'%.0f')];
    end

end


%% Sending commands to lynx
if(nargin == 6)
    for(i = 1:6)
        fprintf(lynx.serialPort, '%s\r',[str{i}]);
    end
end

end

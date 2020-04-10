function [qNext, isDone] = potentialFieldStep_groupno(qCurr, map, robot)
% POTENTIALFIELDSTEP_GROUPNO Calculates a single step in a potential field
%   planner based on the virtual forces exerted by all of the elements in
%   map. This function will be called over and over until isDone is set.
%   Use persistent variables if you need historical information. CHANGE 
%   GROUPNO TO YOUR GROUP NUMBER.
%
% INPUTS:
%   qCurr - 1x6 vector representing the current configuration of the robot.
%   map   - a map struct containing the boundaries of the map, any
%           obstacles, the start position, and the goal position.
%   robot - a struct of robot parameters
%
% OUTPUTS:
%   qNext  - 1x6 vector representing the next configuration of the robot
%            after it takes a single step along the potential field.
%   isDone - a boolean flag signifying termination of the potential field
%            algorithm. 
%
%               isDone == 1 -> Terminate the planner. We have either
%                              reached the goal or are stuck with no 
%                              way out.
%               isDone == 0 -> Keep going.

%%

qNext = zeros(1,6);
qNext = qCurr;
% qNext(1) = qNext(1)+.01;
isDone = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algorithm Starts Here             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[row, dof] = size(qCurr); % get the dof 
[obsNum, col] = size(map.obstacles);
repThreshold = 50; % repulsive threshold distance set to be 40.
qCriteria = 0.1; % criteria for judging the difference/2-norm distance in configuration space for angles. 
qGoal = map.goal;
qDistance = norm(qCurr - qGoal); % use 2-norm to get the distance

repForce = zeros(dof + 1, 3); % initialize a NX3 repulsive force matrix, each row represent the force vector for ith joint
attForce = zeros(dof + 1, 3); % initialize a NX3 attractive force matrix, each row represent the force vector for ith joint
totalForce = zeros(dof + 1, 3); % initialize a NX3 total force for combining repulsive and attractive force
torque = zeros(dof,1); % initialize a NX1 array of torque for each joint.
repStrength = 1000; % set repulsive field strength to be 0.3
attStrength = 0.01; % set attractive field strength to be 0.2

configStep = 0.01; % initialize a fixed-size step in joint space.

isDone = 1; % let isDone to be true before entering the if loop. If it does not enter --> means meet the criteria so isDone = 1
if qDistance > qCriteria
    isDone = 0; % if enters the if loop, then the current position is not inside the criteria, so iterate again for the next time to judge
   
    % perform calculation for each obstacles
    for k = 1: obsNum
        %------start getting the distance of current position-----
        % get the current joint position first from FK
        [currPos,T0i] = calculateFK_sol(qCurr, robot);
        [dist, unit] = distPointToBox(currPos, map.obstacles(k,:));
        %-----finish getting the distance-------------------------
        
        %-----start calculating the repulsive and attractive force for every joints-------
        % get the final position of the joints in workspace
        [goalPos, T0i_final] = calculateFK_sol(qGoal, robot);
        for i = 1: (dof + 1)
            if dist(i) < repThreshold
                repForce(i, :) = repStrength * (1/dist(i) - 1/repThreshold) * (1/(dist(i)^2)) * -1 * unit(i, :);
            else
                repForce(i, :) = 0; % repForce = 0 if outside of influence
            end
            attForce(i, :) = -1 * attStrength * (currPos(i, :) - goalPos(i, :)); 
        end
        %-----finish calculating the repulsive and attractive force for every joints-------
        
        % sum up the repulsive force and attrative force from each obstacle
        totalForce = (repForce + attForce) + totalForce;
    end
    
    %-----start calculating the torque for each joints
    for i = 2: (dof + 1)
        for j = 2:i
        J = calcJacobian_group37(qCurr, i, robot); % get Jacobian
        tau = forceToTorque_group37([totalForce(i,:), 0, 0, 0], J); % calculate torque
        torque(j-1) = torque(j-1) + tau(j-1); % sum up the torque induced from each joint force.
        end
    end
    
    %------start getting the updated joint position for the next round-----
    for i = 1:dof
        qNext(i) = qNext(i) + configStep * (torque(i)/norm(torque));
    end
    
    % introduce random walk
    % qNext = qNext + 0.01 * rand(1,6);
    
    
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algorithm Ends Here               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end
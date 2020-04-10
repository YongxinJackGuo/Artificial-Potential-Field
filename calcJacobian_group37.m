function J = calcJacobian_group37(q, joint, robot)
% CALCJACOBIAN_GROUPNO Calculate the Jacobian of a particular joint of the 
%   robot in a given configuration. CHANGE GROUPNO TO YOUR GROUP NUMBER.
%
% INPUTS:
%   q     - 1 x 6 vector of joint inputs [q1,q2,q3,q4,q5,q6]
%   joint - scalar in [1,7] representing which joint we care about
%   robot - a struct of robot parameters
%
% OUTPUTS:
%   J - 6 x (joint-1) matrix representing the Jacobian
%

%%

J = [];

if nargin < 3
    return
elseif joint <= 1 || joint > 7
    return
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Your Code Starts Here             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
J = zeros(6,joint-1);

% Homogeneous transformation matrix for each frame
[jointPositions,T0i] = calculateFK_sol(q, robot);

% Calculate the Zs and Os
O = jointPositions;

Jv_geo = [];
T0k_all = [];

%%
for n = 1:1:joint-1         % for revolute joint 1, 2, 3, 4, 5 and 6
    % O6-O(i-1), but the index of Matlab start from 1
    delta_O = O(joint, 1:3) - O(n, 1:3);
    
    % z(i-1), but the index of Matlab start from 1
    T0k = T0i(1:3, 3, n);
    
    Jv_geo = [Jv_geo, cross(T0k, delta_O')];
    
    T0k_all = [T0k_all T0k];
end

Jv = Jv_geo;

%% the angular velocity Jacobian would be the set of Z axis
Jw = T0k_all;

J = [Jv; Jw];
fprintf('\n');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Your Code Ends Here               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end
function tau = forceToTorque_group37(F, J)
% FORCETOTORQUE_GROUPNO Calculate the joint torques required to exert a 
%   specific set of end-effector forces/torques in a given configuration.
%   Note that you can write this function entirely generally, with no
%   reference to the Lynx. CHANGE GROUPNO TO YOUR GROUP NUMBER.
%
% INPUTS:
%   F - 1 x 6 vector of desired forces/torques (where F(1:3) is the
%       forces and F(4:6) is the torques)
%   J - the 6xN jacobian of the robot in its current configuration  
%
% OUTPUTS:
%   tau - Nx1 vector of joint torques required to generate F.

%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Your Code Starts Here             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tau = transpose(J) * transpose(F);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Your Code Ends Here               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end
function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here

% arm tool position control
% always active
uvms.A.t = eye(6);

% vehicle position and orientation control
uvms.A.v_l = eye(3);
uvms.A.v_a = eye(3);

% horizontal attitude
uvms.A.ha = IncreasingBellShapedFunction(0.2, 0.4, 0, 1, norm(uvms.v_rho_ha) );

% minimum altitude 
uvms.A.ma = DecreasingBellShapedFunction(uvms.min_alt_value + 1, uvms.min_alt_value + 2, 0, 1, uvms.a );


end
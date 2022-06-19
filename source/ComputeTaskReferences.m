function [uvms] = ComputeTaskReferences(uvms, mission)
% compute the task references here

% reference for tool-frame position control task
[ang, lin] = CartError(uvms.vTg , uvms.vTt);
% uvms.xdot.t = 0.2 * [ang; lin];
uvms.xdot.t = 0.5 * [ang; lin];
% limit the requested velocities...
uvms.xdot.t(1:3) = Saturate(uvms.xdot.t(1:3), 0.2);
uvms.xdot.t(4:6) = Saturate(uvms.xdot.t(4:6), 0.2);
% mission
uvms.tool_lin = norm( lin );

% ---

% joint limit costraint
uvms.xdot.cjoint = 0.2 * ( uvms.q_m - uvms.q );

% zero velocity constraint
uvms.xdot.zero = 0.5 * ( zeros(6, 1) - uvms.p_dot );

% reference for vehicle position control task
[ang_v, lin_v] = CartError(uvms.wTgv , uvms.wTv);
uvms.xdot.v_l = 0.5 * lin_v;
uvms.xdot.v_a = 0.5 * ang_v;
uvms.xdot.v_l = Saturate(uvms.xdot.v_l, 0.5);
uvms.xdot.v_a = Saturate(uvms.xdot.v_a, 0.5);

% reference for the horizontal attitude task
uvms.xdot.ha = 0.5 *(0 - norm(uvms.v_rho_ha));

% preferred shape task
uvms.xdot.sh = 0.5 * ( uvms.q_sh - uvms.q(1:4) );

% target alignment task
% uvms.xdot.align = 0.5 * (0 - norm( uvms.w_rho_align ));

% reference for the minimum altitude task
% uvms.xdot.ma = 0.2 * (1 - uvms.a);

% reference for the landing action 
% uvms.xdot.a = 0.3 * (0 - uvms.a);

% ---

end
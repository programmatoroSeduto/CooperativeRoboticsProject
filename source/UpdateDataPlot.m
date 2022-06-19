function [ plt ] = UpdateDataPlot( plt, uvms, t, loop )

% PLOT -- activation functions
% minimum altitude
plt.A.min_alt(:, loop) = uvms.A.ma;
% horizontal attitude
plt.A.hor_ctrl(:, loop) = uvms.A.ha; 
% landing action
plt.A.land(:, loop) = uvms.A.a;
% position control task
plt.A.pos(:, loop) = [ uvms.A.v_l(1, 1) uvms.A.v_l(2, 2) uvms.A.v_l(3, 3) ]';
% orientation control task
plt.A.orient(:, loop) = [ uvms.A.v_a(1, 1) uvms.A.v_a(2, 2) uvms.A.v_a(3, 3) ]';

% PLOT -- position and orientation
% position
plt.pos(:, loop) = uvms.p( 1:3 );
% orientation
plt.orient(:, loop) = uvms.p( 4:6 );

% PLOT -- dot position and dot orientation
% dot position
plt.dot_pos(:, loop) = uvms.p_dot( 1:3 );
% dot orientation
plt.dot_orient(:, loop) = uvms.p_dot( 4:6 );

end
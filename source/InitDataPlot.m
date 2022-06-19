function [ plt ] = InitDataPlot( maxloops, deltat, end_time )

% general data
% plt.t = zeros(1, maxloops);
plt.t = 0:deltat:end_time;
plt.end_time = end_time;

% PLOT -- activation functions
% zero velocity constraint
plt.A.zero = zeros(1, maxloops);
% minimum altitude
plt.A.min_alt = zeros(1, maxloops);
% horizontal attitude
plt.A.hor_ctrl = zeros(1, maxloops);
% landing action
plt.A.land = zeros(1, maxloops);
% position control task
plt.A.pos = zeros(3, maxloops);
% orientation control task
plt.A.orient = zeros(3, maxloops);
% vehicle alignment to a target
plt.A.align = zeros(1, maxloops);
% tool frame motion
plt.A.t = zeros(6, maxloops);

% PLOT -- manupulator tool frame pos and orientaton
% position
plt.e_pos = zeros(3, maxloops);
% orientation
% plt.e_orient = zeros(3, maxloops);

% PLOT -- manipulator configuration
% configuration
plt.q = zeros( 7, maxloops );
% derivative of the configuration vector
plt.q_dot = zeros( 7, maxloops );

% PLOT -- position and orientation
% position
plt.pos = zeros(3, maxloops);
% orientation
plt.orient = zeros(3, maxloops);

% PLOT -- dot position and dot orientation
% dot position
plt.dot_pos = zeros(3, maxloops);
% dot orientation
plt.dot_orient = zeros(3, maxloops);

end


function [ plt ] = InitDataPlot( maxloops, deltat, end_time )

% general data
% plt.t = zeros(1, maxloops);
plt.t = 0:deltat:end_time;
plt.end_time = end_time;

% PLOT -- activation functions
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


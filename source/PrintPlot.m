function [ ] = PrintPlot( plt )

idx_plot = 0;

% PLOT -- manipulator joint limit constrant
idx_plot = idx_plot + 1;
figure( idx_plot ), grid on;
title( "Joint Limit Constraint", "activation functions" );
hold on;
plot( plt.t, plt.A.cjoint );
hold off;
legend( ...
    'a_j1', ...
    'a_j2', ...
    'a_j3', ...
    'a_j4', ...
    'a_j5', ...
    'a_j6', ...
    'a_j7' )


% PLOT -- all the activations 
idx_plot = idx_plot + 1;
figure( idx_plot ), grid on;
% title( "plot(" + idx_plot + ") " + "Activation Functions", "All the activation functions" );
title( "Activation Functions", "All the activation functions" );
hold on;
% plot( [0, plt.end_time], [1, 1], ':' );
plot( plt.t, plt.A.zero, ':g' );
% plot( plt.t, plt.A.min_alt, '-b' );
plot( plt.t, plt.A.hor_ctrl, '-r' );
% plot( plt.t, plt.A.align, '-m' );
% plot( plt.t, plt.A.land, '-.b' );
plot( plt.t, plt.A.pos, '--b' );
plot( plt.t, plt.A.orient, '--r' );
plot( plt.t, plt.A.t, '-.k' );
hold off;
ylim( [0, 1.1] )
legend( ...
    'zero velocity constraint', ...
    'horizontal attitude', ...
    'vehicle position control -- x', ...
    '-- y', ...
    '-- z', ...
    'vehicle orientation control -- roll', ...
    '-- pitch', ...
    '-- yaw', ...
    'manipulator task' )


% PLOT -- minimum altitude and altitude control task
%{
idx_plot = idx_plot + 1;
figure( idx_plot ), grid on;
% title( "plot(" + idx_plot + ") " + "Activation Functions", "minimum altitude Vs. altitude control task" );
title( "Activation Functions", "minimum altitude Vs. altitude control task" );
hold on;
plot( plt.t, plt.A.min_alt, '-b' );
plot( plt.t, plt.A.land, '--b' );
hold off;
legend( ...
    'minimum altitude', ...
    'zero altitude' )
%}

% PLOT -- end effector position
idx_plot = idx_plot + 1;
figure( idx_plot ), grid on;
title( "End Effector Cartesian Coordinates", "Coordinates of the manipuator tip wrt frame <w>" );
hold on;
plot( plt.t, plt.e_pos(1, :), '-r' );
plot( plt.t, plt.e_pos(2, :), '-b' );
plot( plt.t, plt.e_pos(3, :), '-k' );
hold off;
legend( ...
    'Xe', ...
    'Ye', ...
    'Ze' );


% PLOT -- (row 1) position and (row 2) orientation
idx_plot = idx_plot + 1;
figure( idx_plot );
subplot( 2, 1, 1 );
hold on, grid on;
plot( plt.t, plt.pos(1, :), '-r' );
plot( plt.t, plt.pos(2, :), '-b' );
plot( plt.t, plt.pos(3, :), '-k' );
title( "position" )
legend( 'x', 'y', 'z' );
hold off;
subplot( 2, 1, 2 );
hold on, grid on;
plot( plt.t, plt.orient(1, :), '-r' );
plot( plt.t, plt.orient(2, :), '-b' );
plot( plt.t, plt.orient(3, :), '-k' );
title( "orientation" )
legend( 'roll (x)', 'pitch (y)', 'yaw (z)' );
hold off;
% sgtitle( "plot(" + idx_plot + ") " + "Vehicle Position and Orientation" );
sgtitle( "Vehicle Position and Orientation" );


% PLOT -- (row 1) dot position and (row 2) dot orientation
idx_plot = idx_plot + 1;
figure( idx_plot );
subplot( 2, 1, 1 );
hold on, grid on;
plot( plt.t, plt.dot_pos(1, :), '-r' );
plot( plt.t, plt.dot_pos(2, :), '-b' );
plot( plt.t, plt.dot_pos(3, :), '-k' );
title( "dot position" )
legend( 'Xdot', 'Ydot', 'Zdot' );
hold off;
subplot( 2, 1, 2 );
hold on, grid on;
plot( plt.t, plt.dot_orient(1, :), '-r' );
plot( plt.t, plt.dot_orient(2, :), '-b' );
plot( plt.t, plt.dot_orient(3, :), '-k' );
title( "dot orientation" )
legend( 'Wx', 'Wy', 'Wx' );
hold off;
% sgtitle( "plot(" + idx_plot + ") " + "Vehicle dotPosition and dotOrientation" );
sgtitle( "Vehicle dotPosition and dotOrientation" );


% PLOT -- vehicle position Vs end effector position
idx_plot = idx_plot + 1;
figure( idx_plot );
subplot( 2, 1, 1 );
hold on, grid on;
plot( plt.t, plt.dot_pos(1, :), ':b' );
plot( plt.t, plt.dot_pos(2, :), '-b' );
plot( plt.t, plt.dot_pos(3, :), '--b' );
% plot( plt.t, plt.orient(1, :), ':r' );
% plot( plt.t, plt.orient(2, :), '-r' );
% plot( plt.t, plt.orient(3, :), '--r' );
title( "dot position" )
% legend( 'Xdot', 'Ydot', 'Zdot', 'Wx', 'Wy', 'Wz' );
legend( 'Xdot', 'Ydot', 'Zdot' );
hold off;
subplot( 2, 1, 2 );
hold on, grid on;
plot( plt.t, plt.q_dot );
title( "configuration rate" )
legend( 'qdot_1', 'qdot_2', 'qdot_3', 'qdot_4', 'qdot_5', 'qdot_6', 'qdot_7' );
hold off;
% sgtitle( "plot(" + idx_plot + ") " + "Vehicle dotPosition and dotOrientation" );
sgtitle( "Vehicle velocity Vs. Manipulator motion rate" );

end


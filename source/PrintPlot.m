function [ ] = PrintPlot( plt )

idx_plot = 1;

% PLOT -- all the activations 
figure( idx_plot ), grid on;
% title( "plot(" + idx_plot + ") " + "Activation Functions", "All the activation functions" );
title( "Activation Functions", "All the activation functions" );
hold on;
% plot( [0, plt.end_time], [1, 1], ':' );
plot( plt.t, plt.A.min_alt, '-b' );
plot( plt.t, plt.A.hor_ctrl, '-r' );
plot( plt.t, plt.A.align, '-m' );
plot( plt.t, plt.A.land, '-.b' );
plot( plt.t, plt.A.pos, '--b' );
plot( plt.t, plt.A.orient, '--r' );
plot( plt.t, plt.A.t, '-.k' );
hold off;
ylim( [0, 1.1] )
legend( ...
    'minimum altitude', ...
    'horizontal attitude', ...
    'alignment to the target', ...
    'zero altitude', ...
    'vehicle position control -- x', ...
    '-- y', ...
    '-- z', ...
    'vehicle orientation control -- roll', ...
    '-- pitch', ...
    '-- yaw', ...
    'manipulator task' )

% PLOT -- minimum altitude and altitude control task
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

end


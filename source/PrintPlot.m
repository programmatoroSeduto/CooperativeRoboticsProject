function [ ] = PrintPlot( plt )

%{
figure(1);
subplot(2,1,1);
hplot = plot(plt.t, plt.q);
set(hplot, 'LineWidth', 1);
legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
subplot(2,1,2);
hplot = plot(plt.t, plt.q_dot);
set(hplot, 'LineWidth', 1);
legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7');

figure(2);
subplot(3,1,1);
hplot = plot(plt.t, plt.p);
set(hplot, 'LineWidth', 1);
legend('x','y','z','roll','pitch','yaw');
subplot(2,1,2);
hplot = plot(plt.t, plt.p_dot);
set(hplot, 'LineWidth', 1);
legend('xdot', 'ydot','zdot','omega_x','omega_y','omega_z');

figure(3);
hplot = plot(plt.t, plt.a(1:7,:));
set(hplot, 'LineWidth', 2);
legend('Ajl_11','Ajl_22','Ajl_33','Ajl_44','Ajl_55','Ajl_66','Ajl_77');

figure(4);
hplot = plot(plt.t, plt.a(8:9,:));
set(hplot, 'LineWidth', 2);
legend('Amu', 'Aha');
%}

idx_plot = 1;

% PLOT -- all the activations 
figure( idx_plot ), grid on;
title( "plot(" + idx_plot + ") " + "Activation Functions", "All the activation functions" );
hold on;
% plot( [0, plt.end_time], [1, 1], ':' );
plot( plt.t, plt.A.min_alt, '-b' );
plot( plt.t, plt.A.hor_ctrl, '-r' );
plot( plt.t, plt.A.land, '-.b' );
plot( plt.t, plt.A.pos, '--b' );
plot( plt.t, plt.A.orient, '--r' );
hold off;
ylim( [0, 1.1] )
legend( ...
    'minimum altitude', ...
    'horizontal attitude', ...
    'zero altitude', ...
    'vehicle position control -- x', ...
    '-- y', ...
    '-- z', ...
    'vehicle orientation control -- roll', ...
    '-- pitch', ...
    '-- yaw' )

% PLOT -- minimum altitude and altitude control task
idx_plot = idx_plot + 1;
figure( idx_plot ), grid on;
title( "plot(" + idx_plot + ") " + "Activation Functions", "minimum altitude Vs. altitude control task" );
hold on;
plot( plt.t, plt.A.min_alt, '-b' );
plot( plt.t, plt.A.land, '--b' );
hold off;
legend( ...
    'minimum altitude', ...
    'zero altitude' )

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
sgtitle( "plot(" + idx_plot + ") " + "Vehicle Position and Orientation" );

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
sgtitle( "plot(" + idx_plot + ") " + "Vehicle dotPosition and dotOrientation" );

end


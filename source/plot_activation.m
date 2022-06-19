clc, clear, close all;
addpath('./simulation_scripts');
n_points = 500;
q = linspace( 0, 10, n_points );
qm = 2.3;
e = 1.2;
a = zeros( 1, n_points );

for i=1:n_points
    if ( q(i) >= qm - e ) && ( q(i) < qm )
        a(i) = DecreasingBellShapedFunction( qm - e, qm, 0, 1, q(i) );
	elseif ( q(i) < qm + e ) && ( q(i) >= qm )
        a(i) = IncreasingBellShapedFunction( qm, qm + e, 0, 1, q(i) );
    else
        a(i) = 1;
    end
end

figure, box on, grid on, hold on;
title( "Joint Limit Constraint Activation Function", ...
    "Shape of the activation function with q_m=" + qm + " and eps=" + e + "" )
xlim( [0, 6] )
ylim( [-0.1, 1.5] )
plot( q, a, 'r' );
plot( [ qm-e, qm-e ], [ -2, 2 ], '--b' )
plot( [ qm, qm ], [ -2, 2 ], '--g' )
plot( [ qm+e, qm+e ], [ -2, 2 ], '--b' )
legend( "activation", "qm-e, qm+e", "qm" )
hold off;
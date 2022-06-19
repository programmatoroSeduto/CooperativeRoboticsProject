function [uvms] = ComputeActivationFunctions(uvms, mission)

uvms = ActivationMission( uvms, mission );

% joint limit constraint
uvms.A.cjoint = eye( 7, 7 ) * uvms.Ap.cjoint;
if uvms.Ap.cjoint > 0
    q = .0;
    qm = .0;
    e = .0;
    for i=1:7
        q = abs( uvms.q(i) );
        qm = abs( uvms.q_m(i) );
        e = uvms.eps( i );
        if ( q >= qm - e ) && ( q < qm )
            uvms.A.cjoint(i, i) = uvms.A.cjoint(i, i) * ...
                DecreasingBellShapedFunction( qm - e, qm, 0, 1, q );
        elseif ( q < qm + e ) && ( q >= qm )
            uvms.A.cjoint(i, i) = uvms.A.cjoint(i, i) * ...
                IncreasingBellShapedFunction( qm, qm + e, 0, 1, q );
        else
            uvms.A.cjoint(i, i) = uvms.Ap.cjoint;
        end
    end
end

% zero velocity constraint
uvms.A.zero = eye(6) * uvms.Ap.zero;

% arm tool position control
% always active
uvms.A.t = eye(6) * uvms.Ap.t;

% vehicle position and orientation control
uvms.A.v_l = eye(3)*uvms.Ap.v_l;
uvms.A.v_a = eye(3)*uvms.Ap.v_a;

% horizontal attitude
uvms.A.ha = IncreasingBellShapedFunction(0.1, 0.3, 0, 1, norm(uvms.v_rho_ha))*uvms.Ap.ha;

% preferred shape task
uvms.A.sh = eye( 4 ) * uvms.Ap.sh;
if uvms.Ap.sh > .0
    q = .0;
    qm = .0;
    e = uvms.eps_sh;
    for i=1:4
        q = abs( uvms.q(i) );
        qm = abs( uvms.q_sh(i) );
        if ( q >= qm - e ) && ( q < qm )
            uvms.A.sh(i, i) = uvms.Ap.sh * ...
                DecreasingBellShapedFunction( qm - e, qm, 0, 1, q );
        elseif ( q < qm + e ) && ( q >= qm )
            uvms.A.sh(i, i) = uvms.Ap.sh * ...
                IncreasingBellShapedFunction( qm, qm + e, 0, 1, q );
        else
            uvms.A.sh(i, i) = uvms.Ap.sh;
        end
    end
end

% alignment task
% uvms.A.align = IncreasingBellShapedFunction(0.001, 0.05, 0, 1, norm(uvms.w_rho_align))*uvms.Ap.align;

% minimum altitude 
% uvms.A.ma = DecreasingBellShapedFunction(0.5, 1, 0, 1, uvms.a )*uvms.Ap.ma;

% landing action
% uvms.A.a = 1*uvms.Ap.a;

end
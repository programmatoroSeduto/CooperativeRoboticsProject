function [uvms] = ComputeActivationFunctions(uvms, mission)

if mission.mission_on
    switch mission.phase
        case mission.ph.a1 % reach the target point
            uvms.Ap.a = 0;
            uvms.Ap.ma = 1;
            uvms.Ap.ha = 1;
            uvms.Ap.align = 0;
            uvms.Ap.v_l = 1;
            uvms.Ap.v_a = 1;

        case mission.ph.a1_to_a2 % transient from A1 to A2
            uvms.Ap.a = IncreasingBellShapedFunction( 0, mission.a1_to_a2.trans_time, 0, 1, mission.phase_time );
            uvms.Ap.ma = DecreasingBellShapedFunction( 0, mission.a1_to_a2.trans_time, 0, 1, mission.phase_time );
            uvms.Ap.ha = 1;
            uvms.Ap.align = IncreasingBellShapedFunction( 0, mission.a1_to_a2.trans_time, 0, 1, mission.phase_time );
            uvms.Ap.v_l = DecreasingBellShapedFunction( 0, mission.a1_to_a2.trans_time, 0, 1, mission.phase_time );
            uvms.Ap.v_a = DecreasingBellShapedFunction( 0, mission.a1_to_a2.trans_time, 0, 1, mission.phase_time );

        case mission.ph.a1 % landing phase
            uvms.Ap.v_l = zeros(3);
            uvms.Ap.v_a = zeros(3);
            uvms.Ap.ha = 1;
            uvms.Ap.ma = 0;
            uvms.Ap.a = 1;
            uvms.Ap.align = 1;

        case mission.ph.stop % end of the mission
            uvms.Ap.v_l = 0;
            uvms.Ap.v_a = 0;
            uvms.Ap.ha = 0;
            uvms.Ap.ma = 0;
            uvms.Ap.a = 0;
            uvms.Ap.align = 0;
            
    end
else
    uvms.Ap.a = 0;
    uvms.Ap.ma = 0;
    uvms.Ap.ha = 0;
    uvms.Ap.align = 1;
    uvms.Ap.v_l = 0;
    uvms.Ap.v_a = 0;
end

% arm tool position control
% always active
uvms.A.t = eye(6);

% vehicle position and orientation control
uvms.A.v_l = eye(3)*uvms.Ap.v_l;
uvms.A.v_a = eye(3)*uvms.Ap.v_a;

% horizontal attitude
uvms.A.ha = IncreasingBellShapedFunction(0.2, 0.4, 0, 1, norm(uvms.v_rho_ha))*uvms.Ap.ha;

% alignment task
uvms.A.align = IncreasingBellShapedFunction(0.001, 0.05, 0, 1, norm(uvms.w_rho_align))*uvms.Ap.align;

% minimum altitude 
uvms.A.ma = DecreasingBellShapedFunction(0.5, 1, 0, 1, uvms.a )*uvms.Ap.ma;

% landing action
uvms.A.a = 1*uvms.Ap.a;

end
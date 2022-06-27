function [uvms] = ActivationMission(uvms, mission)

if mission.mission_on
    switch mission.phase
        case mission.ph.start 
            uvms.Ap.cjoint = 0;
            uvms.Ap.zero = 0;
            uvms.Ap.ha = 0;
            % uvms.Ap.align = 0;
            uvms.Ap.t = 0;
            uvms.Ap.v_l = 0;
            uvms.Ap.v_a = 0;
        case mission.ph.safe_waypoint_nav
            uvms.Ap.cjoint = 0;
            uvms.Ap.zero = 0;
            uvms.Ap.ha = 1;
            % uvms.Ap.align = 0;
            uvms.Ap.t = 0;
            uvms.Ap.v_l = 1;
            uvms.Ap.v_a = 1;
        case mission.ph.trans_nav
            uvms.Ap.cjoint = IncreasingBellShapedFunction( 0, mission.trans_nav.trans_time, 0, 1, mission.phase_time );
            uvms.Ap.zero = IncreasingBellShapedFunction( 0, mission.trans_nav.trans_time, 0, 1, mission.phase_time );
            uvms.Ap.ha = 1;
            % uvms.Ap.align = 0;
            uvms.Ap.t = IncreasingBellShapedFunction( 0, mission.trans_nav.trans_time, 0, 1, mission.phase_time );
            uvms.Ap.v_l = DecreasingBellShapedFunction( 0, mission.trans_nav.trans_time, 0, 1, mission.phase_time );
            uvms.Ap.v_a = DecreasingBellShapedFunction( 0, mission.trans_nav.trans_time, 0, 1, mission.phase_time );
        case mission.ph.manipulation
            uvms.Ap.cjoint = 1;
            uvms.Ap.zero = 1;
            uvms.Ap.ha = 1;
            % uvms.Ap.align = 0;
            uvms.Ap.t = 1;
            uvms.Ap.v_l = 0;
            uvms.Ap.v_a = 0;
        case mission.ph.stop
            uvms.Ap.cjoint = 0;
            uvms.Ap.zero = 0;
            uvms.Ap.ha = 0;
            % uvms.Ap.align = 0;
            uvms.Ap.t = 0;
            uvms.Ap.v_l = 0;
            uvms.Ap.v_a = 0;
    end
else
    uvms.Ap.cjoint = 0;
    uvms.Ap.zero = 1;
    uvms.Ap.ma = 0;
    % uvms.Ap.align = 0;
    uvms.Ap.v_l = 0;
    uvms.Ap.v_a = 0;
    uvms.Ap.t = 1;
end

end


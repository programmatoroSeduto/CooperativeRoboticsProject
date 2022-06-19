function [uvms, mission] = UpdateMissionPhase( uvms, mission )

mission.phase_time = mission.phase_time + mission.deltat;

if mission.mission_on
    switch mission.phase
        case mission.ph.start
            % set the next status
            mission.phase = mission.ph.safe_waypoint_nav;
            mission.phase_time = 0;
            
            mission.manipulation.dist = Inf;
            mission.safe_waypoint_nav.cart_distance = Inf;
            mission.safe_waypoint_nav.misalignment = Inf;
            
            disp( "Mission Start" )
            
        case mission.ph.safe_waypoint_nav
            % update mission status
            mission.safe_waypoint_nav.cart_distance = norm( 2*uvms.xdot.v_l );
            mission.safe_waypoint_nav.misalignment  = norm( 2*uvms.xdot.v_a );

            % check for the next status
            if ( mission.safe_waypoint_nav.cart_distance <= mission.safe_waypoint_nav.target_dist ) && ...
                    ( mission.safe_waypoint_nav.misalignment <= mission.safe_waypoint_nav.target_ang )
                % notify the phase change
                disp("MISSION :safe_waypoint_nav -> trans_nav (in " + mission.phase_time + "s)");

                % change mission phase
                mission.phase = mission.ph.trans_nav;
                % reset mission time
                mission.phase_time = 0;
                
                mission.safe_waypoint_nav.cart_distance = Inf;
                mission.safe_waypoint_nav.misalignment = Inf;
            end

        case mission.ph.trans_nav
            if( mission.phase_time >= mission.trans_nav.trans_time )
                % notify the phase change
                disp("MISSION : trans_nav -> manipulation (in " + mission.phase_time + "s)");

                % change mission phase
                mission.phase = mission.ph.manipulation;
                % reset mission time
                mission.phase_time = 0;
            end

        case mission.ph.manipulation
            % update mission status
            [~, mission.manipulation.dist] = CartError(uvms.vTg , uvms.vTt);
            
            % check for transition
            if( mission.manipulation.dist <= mission.manipulation.threshold )
                % notify the phase change
                disp("MISSION : a3 -> STOP (in " + mission.phase_time + "s)");

                % change mission phase
                mission.phase = mission.ph.stop;
                % reset mission time
                mission.phase_time = 0;

                % reset unused fields 
                mission.manipulation.dist = Inf;
            end

        case mission.ph.stop % stop phase
            % nothing
    end
end

end


function [uvms, mission] = UpdateMissionPhase( uvms, mission )

mission.phase_time = mission.phase_time + mission.deltat;

if mission.mission_on
    switch mission.phase
        case mission.ph.start
            % init the mission
            mission.a1.cart_distance = Inf;
            mission.a1.misalignment = Inf; 
            mission.a2.altitude = Inf;
            mission.a3.dist = Inf;
            
            % set the next status
            mission.phase = mission.ph.a1;
            mission.phase_time = 0;
            
        case mission.ph.a1 % reach the target point
            % update mission status
            mission.a1.cart_distance = norm( 2*uvms.xdot.v_l );
            mission.a1.misalignment  = norm( 2*uvms.xdot.v_a );

            % check for the next status
            if ( mission.a1.cart_distance <= mission.a1.target_dist ) && ...
                    ( mission.a1.misalignment <= mission.a1.target_ang )
                % notify the phase change
                disp("MISSION : a1 -> a1_to_a2 (in " + mission.phase_time + "s)");

                % change mission phase
                mission.phase = mission.ph.a1_to_a2;
                % reset mission time
                mission.phase_time = 0;

                % reset unused fields 
                mission.a1.misalignment = Inf;
                mission.a1.cart_distance = Inf;
            end

        case mission.ph.a1_to_a2 % transient from A1 to A2
            if( mission.phase_time >= mission.a1_to_a2.trans_time )
                % notify the phase change
                disp("MISSION : a1_to_a2 -> a2 (in " + mission.phase_time + "s)");

                % change mission phase
                mission.phase = mission.ph.a2;
                % reset mission time
                mission.phase_time = 0;
            end

        case mission.ph.a2 % landing phase
            % update phase status
            mission.a2.altitude = uvms.a;

            % check for transition
            if( mission.a2.altitude <= mission.a2.z_threshold )
                % notify the phase change
                disp("MISSION : a2 -> a2_to_a3 (in " + mission.phase_time + "s)");

                % change mission phase
                mission.phase = mission.ph.a2_to_a3;
                % reset mission time
                mission.phase_time = 0;

                % reset unused fields 
                mission.a2.altitude = Inf;
            end
        
        case mission.ph.a2_to_a3 % transition from landing to grasping
            if( mission.phase_time >= mission.a2_to_a3.trans_time )
                % notify the phase change
                disp("MISSION : a2_to_a3 -> a3 (in " + mission.phase_time + "s)");

                % change mission phase
                mission.phase = mission.ph.a3;
                % reset mission time
                mission.phase_time = 0;
            end
        
        case mission.ph.a3 % grasping phaseù
            % update mission status
            [~, mission.a3.dist] = CartError(uvms.vTg , uvms.vTt);
            
            % check for transition
            if( mission.a3.dist <= mission.a3.threshold )
                % notify the phase change
                disp("MISSION : a3 -> STOP (in " + mission.phase_time + "s)");

                % change mission phase
                mission.phase = mission.ph.stop;
                % reset mission time
                mission.phase_time = 0;

                % reset unused fields 
                mission.a3.dist = Inf;
            end

        case mission.ph.stop % stop phase
            % nothing
    end
end

end


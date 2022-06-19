function [ m ] = InitMission( deltat )

m.mission_on = true;
% m.mission_on = false;

% contants for the mission phases
idx = 0;
m.ph.start = idx;
idx = idx + 1;
m.ph.safe_waypoint_nav = idx;
idx = idx + 1;
m.ph.trans_nav = idx;
idx = idx + 1;
m.ph.manipulation = idx;
idx = idx + 1;
m.ph.stop = idx;

% mission phase
m.phase = m.ph.start;
% time from the beginning of the action
m.phase_time = 0;
% time between two frames
m.deltat = deltat;

% phase: safe_waypoint_nav 
m.safe_waypoint_nav.cart_distance = Inf;
m.safe_waypoint_nav.misalignment = Inf;
m.safe_waypoint_nav.target_dist = 0.25;
m.safe_waypoint_nav.target_ang = 0.075;

% phase: trans_nav
m.trans_nav.trans_time = 5.0;

% phase: manipulation
m.manipulation.dist = Inf;
m.manipulation.threshold = 0.1;

end


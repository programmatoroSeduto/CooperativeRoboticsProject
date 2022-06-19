function [ m ] = InitMission( deltat )

m.mission_on = true;
% m.mission_on = false;

% contants for the mission phases
idx = 0;
m.ph.start = idx;
idx = idx + 1;
m.ph.a1 = idx;       % reach the target point
idx = idx + 1;
m.ph.a1_to_a2 = idx; % transient from A1 to A2
idx = idx + 1;
m.ph.a2 = idx;       % landing phase
idx = idx + 1;
m.ph.a2_to_a3 = idx; % transient from A2 to A3
idx = idx + 1;
m.ph.a3 = idx;       % grasping phase
idx = idx + 1;
m.ph.stop = idx;     % end of the mission (retry?)

% mission phase
m.phase = m.ph.a1;
% time from the beginning of the action
m.phase_time = 0;
% time between two frames
m.deltat = deltat;

% phase A1 params
m.a1.cart_distance = Inf; % norm distance between <v> origin and Vehicle Target
m.a1.misalignment = Inf;  % norm rho between <v> origin and Vehicle Target
m.a1.target_dist = 0.25; % target norm value for the distance
m.a1.target_ang = 0.075; % target norm value for the misalignment

% transient A1-A2 phase params
m.a1_to_a2.trans_time = 2; % seconds

% phase A2 params
m.a2.altitude = Inf; % meters
m.a2.z_threshold = 0.005; % meters

% transient A2-A3 phase params
m.a2_to_a3.trans_time = 2; % seconds

% phase A3 params
m.a3.dist = Inf;
m.a3.threshold = 0.00001;

end


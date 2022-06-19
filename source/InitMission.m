function [ m ] = InitMission( deltat )

% contants for the mission phases
m.ph.a1 = 0;       % reach the target point
m.ph.a1_to_a2 = 1; % transient from A1 to A2
m.ph.a2 = 2;       % landing phase
m.ph.stop = 3;     % end of the mission (retry?)

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

% transient phase params
m.a1_to_a2.trans_time = 5; % seconds

% phase A2 params
m.a2.altitude = Inf; % meters
m.a2.z_threshold = 0.005; % meters

end


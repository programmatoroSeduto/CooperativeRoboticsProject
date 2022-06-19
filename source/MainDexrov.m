
addpath('./simulation_scripts');
clc;
clear;
close all

% Simulation variables (integration and final time)
deltat = 0.005;
end_time = 45;
loop = 1;
maxloops = ceil(end_time/deltat);

% this struct can be used to evolve what the UVMS has to do
mission = InitMission( deltat );
%{
mission.phase = 1;
mission.phase_time = 0;
%}

% Rotation matrix to convert coordinates between Unity and the <w> frame
% do not change
wuRw = rotation(0,-pi/2,pi/2);
vRvu = rotation(-pi/2,0,-pi/2);

% pipe parameters
u_pipe_center = [-10.66 31.47 -1.94]'; % in unity coordinates
pipe_center = wuRw'*u_pipe_center;     % in world frame coordinates
pipe_radius = 0.3;

% UDP Connection with Unity viewer v2
uArm = udp('127.0.0.1',15000,'OutputDatagramPacketSize',28);
uVehicle = udp('127.0.0.1',15001,'OutputDatagramPacketSize',24);
fopen(uVehicle);
fopen(uArm);
% ---
% uAltitude = dsp.UDPReceiver('LocalIPPort',15003,'MessageDataType','single');
% uAltitude.setup();
% ---

% Preallocation
plt = InitDataPlot(maxloops, deltat, end_time);

% initialize uvms structure
uvms = InitUVMS('DexROV');
% uvms.q 
% Initial joint positions. You can change these values to initialize the simulation with a 
% different starting position for the arm
uvms.q = [-0.0031 1.2586 0.0128 -1.2460 0.0137 0.0853-pi/2 0.0137]';
% uvms.q = zeros(7, 1);
% uvms.p
% initial position of the vehicle
% the vector contains the values in the following order
% [x y z r(rot_x) p(rot_y) y(rot_z)]
% RPY angles are applied in the following sequence
% R(rot_x, rot_y, rot_z) = Rz (rot_z) * Ry(rot_y) * Rx(rot_x)
uvms.p = [-1.9379 10.4813-6.1 -29.7242-0.1 0 0 0]';
% uvms.p = [ 8.5 38.5 -36 0 -0.06 0.5 ]'; 

% initial goal position definition
% slightly over the top of the pipe
distanceGoalWrtPipe = 0.3;
uvms.goalPosition = pipe_center + (pipe_radius + distanceGoalWrtPipe)*[0 0 1]';
uvms.wRg = rotation(pi,0,0);
uvms.wTg = [uvms.wRg uvms.goalPosition; 0 0 0 1];
%{
uvms.goalPosition = [12.2025   37.3748  -39.8860]';
% uvms.goalPosition = [5.0   37.3748  -33.8860]';
uvms.wRg = rotation(0, pi, pi/2);
uvms.wTg = [uvms.wRg uvms.goalPosition; 0 0 0 1];
%}

% rock position 
% rock_center = [12.2025   37.3748  -39.8860]'; % in world frame coordinates
% rock_center = [5.0   37.3748  -33.8860]'; % in world frame coordinates
% uvms.w_align_target = rock_center;

% defines the goal position for the vehicle position task
% uvms.vehicleGoalPosition = [10.5 37.5 -38]';
% uvms.vehicleGoalPosition = [10.75 37.5 -38]';
uvms.vehicleGoalPosition = uvms.goalPosition + [0 0 1.6]';
uvms.wRgv = rotation(0, -0.06, -0.5);
uvms.wTgv = [uvms.wRgv uvms.vehicleGoalPosition; 0 0 0 1];

% minimum altitude
% uvms.min_alt_value = 0;

% defines the tool control point
uvms.eTt = eye(4);

% joint limit constraint average
uvms.q_m = ( uvms.jlmin + uvms.jlmax ) / 2;
% joint limit neighborhood
uvms.eps = .9 * ( uvms.jlmax - uvms.jlmin );

% preferred shape
uvms.q_sh = [-0.0031 1.2586 0.0128 -1.2460]';
% epsilon for the preferred shape
uvms.eps_sh = .0001;

tic
for t = 0:deltat:end_time
    % update all the involved variables
    uvms = UpdateTransforms(uvms);
    uvms = ComputeJacobians(uvms);
    uvms = ComputeTaskReferences(uvms, mission);
    uvms = ComputeActivationFunctions(uvms, mission);
    
    % ---
    % receive altitude information from unity
    % uvms = ReceiveUdpPackets(uvms, uAltitude);
    % ---
   
    % main kinematic algorithm initialization
    % rhop order is [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
    
    rhop = zeros(13,1);
    Qp = eye(13); 
    
    % ---
    
    % === TPIK 1 (vehicle) ===
    
    % underactuation task
    [Qp, rhop] = iCAT_task(uvms.A.und,    uvms.Jund,  Qp, rhop, uvms.xdot.und,  0.0001,   0.01, 10);
    % zero velocity constraint
    [Qp, rhop] = iCAT_task(uvms.A.zero,    uvms.Jzero,  Qp, rhop, uvms.xdot.zero,  0.0001,   0.01, 10);
    % horizontal attitude
    [Qp, rhop] = iCAT_task(uvms.A.ha,    uvms.Jha,  Qp, rhop, uvms.xdot.ha,  0.0001,   0.01, 10);
    % Position Control Task
    [Qp, rhop] = iCAT_task(uvms.A.v_l,   uvms.Jv_l, Qp, rhop, uvms.xdot.v_l,  0.0001,   0.01, 10);    
    % Orientation Control Task
    [Qp, rhop] = iCAT_task(uvms.A.v_a,   uvms.Jv_a, Qp, rhop, uvms.xdot.v_a,  0.0001,   0.01, 10);
    
    uvms.p_dot_ref = rhop(8:13);
    
    % === TPIK 2 (manipulator) ===
    
    % vehicle constraint velocity
    [Qp, rhop] = iCAT_task(uvms.A.vcv,    uvms.Jvcv,  Qp, rhop, uvms.xdot.vcv,  0.0001,   0.01, 10);
	% joint limit constraint
    [Qp, rhop] = iCAT_task(uvms.A.cjoint,    uvms.Jcjoint,  Qp, rhop, uvms.xdot.cjoint,  0.0001,   0.01, 10);
    % manipulator tip motion
    [Qp, rhop] = iCAT_task(uvms.A.t,     uvms.Jt,   Qp, rhop, uvms.xdot.t,  0.0001,   0.01, 10);
    % preferred shape task
    [Qp, rhop] = iCAT_task(uvms.A.sh,     uvms.Jsh,   Qp, rhop, uvms.xdot.sh,  0.0001,   0.01, 10);
    
    % ---
    
    [Qp, rhop] = iCAT_task(eye(13),     eye(13),    Qp, rhop, zeros(13,1),  0.0001,   0.01, 10);    % this task should be the last one
    
    % get the two variables for integration
    uvms.q_dot = rhop(1:7);
    uvms.p_dot = uvms.p_dot_ref;
    
    % apply disturbances on the pitch
    % uvms.p_dot(5) = uvms.p_dot(5) + 0.002*sin(2*pi*0.5*t);
    % uvms.p_dot(5) = 0.1*sin(2*pi*0.5*t) + 0.01*(2*rand - 1);
    uvms.p_dot(5) = 0.05*sin(2*pi*0.5*t);
    
    % Integration
	uvms.q = uvms.q + uvms.q_dot*deltat;
    % beware: p_dot should be projected on <v>
    uvms.p = integrate_vehicle(uvms.p, uvms.p_dot, deltat);
    
    % check if the mission phase should be changed
    [uvms, mission] = UpdateMissionPhase(uvms, mission);
    
    % send packets to Unity viewer
    SendUdpPackets(uvms,wuRw,vRvu,uArm,uVehicle);
        
    % collect data for plots
    plt = UpdateDataPlot(plt,uvms,t,loop);
    loop = loop + 1;
   
    % add debug prints here
    if (mod(t,0.1) == 0)
        % t
        % uvms.sensorDistance
        % uvms.A.ma
        % uvms.xdot.ma
        % uvms.wTe( 1:3, 4 )
        % uvms.wTv( 1:3, 4 )
        % uvms.A.a
    end
    
    % enable this to have the simulation approximately evolving like real
    % time. Remove to go as fast as possible
    SlowdownToRealtime(deltat);
end

fclose(uVehicle);
fclose(uArm);

PrintPlot(plt);


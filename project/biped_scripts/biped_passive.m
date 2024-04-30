%params
p.robot = biped_robot();
p.NB = p.robot.NB;
p.ngc = numel(p.robot.gc.body);

tstart = 0;
tend = 1.0;

%initial conditions
nq = p.NB;

% joint configuration convention:
% [x,y,z,roll,pitch,yaw]
% [FR_abad, FR_hip, FR_knee]
% [FL_abad, FL_hip, FL_knee]

%start at foot to be at zero height
center_pose = [0;0;0.41;0;0;0];
FR_config = [0;deg2rad(45);deg2rad(-90); deg2rad(45)];
FL_config = [0;deg2rad(45);deg2rad(-90); deg2rad(45)];
q0 = [center_pose; FR_config; FL_config];
dq0 = zeros(nq,1);

% not control input, this is for calculating friction forces
u0 = zeros(2*p.ngc,1);
z0 = [q0;dq0;u0];
[ts, zs] = ode45(@(t,z) dynamics(t,z,p),[tstart,tend], z0);
zs_out = zs(:,1:p.NB);

showmotion(p.robot,ts',zs_out');

function zdot = dynamics(t,z,p)
    % contact parameters (stiffness, damping, and friction coefficient)
    K = 1e6;
    D = 2000;
    mu = 0.5;

    idx = 0;
    q = z(idx+(1:p.NB));
    idx = idx + p.NB;
    dq = z(idx+(1:p.NB));
    idx = idx + p.NB;
    u = z(idx+(1:2*p.ngc));
    u = reshape(u, [2, p.ngc]);


    % contact pos/vel
    posvel = gcPosVel(p.robot,q,dq);
    pos = posvel(1:3,:);
    vel = posvel(4:6,:);


    % ground contact dynamics
    [force, udot, fcone] = gcontact(K, D, mu, pos, vel, u);

    force6D = Fpt(force, pos);

    f_ext = cell(p.NB,1);
    for i = 1:p.NB
        f_ext{i} = zeros(6,1);
    end
    for i = 1:p.ngc
        f_ext{p.robot.gc.body(i)} = f_ext{p.robot.gc.body(i)} + force6D(:,i);
    end

    tau = zeros(p.NB,1);

    ddq = FDcrb(p.robot, q, dq, tau, f_ext);

    zdot = [dq;ddq;udot(:)];
end
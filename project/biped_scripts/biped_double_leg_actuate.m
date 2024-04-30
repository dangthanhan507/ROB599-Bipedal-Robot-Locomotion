%{
    same test as biped_double_leg.m
    but now instead, we will actuate the joints using Inverse Dynamics PD control
%}

p.robot = biped_robot();
p.NB = p.robot.NB;
p.ngc = numel(p.robot.gc.body);
nq = p.NB;

hipOffset = 0.062;
hipLength = 0.209;
kneeLength = 0.209;

bodyWidth = 0.049*2;
bodyLength = 0.049*2;
bodyHeight = 0.05*2;

curr_left_xy = [0.0, hipOffset]';
curr_right_xy = [0.0, hipOffset]';

des_left_xy = [0.1, hipOffset]';
des_right_xy = [0.05, hipOffset]';

height = 0.30;
z_max = 0.03;

swing_time = 0.1;
swing_dt = 0.01;
% left_joints_fn = ik_stance_leg_fn(curr_left_xy, des_left_xy, height, hipOffset, hipLength, kneeLength, true, swing_dt, swing_time);
left_joints_fn = ik_swing_leg_fn(curr_left_xy, des_left_xy, height, z_max, hipOffset, hipLength, kneeLength, true, swing_dt, swing_time);
left_joints_fn = @(t) -1*left_joints_fn(t);

right_joints_fn = ik_stance_leg_fn(curr_right_xy, des_right_xy, height, hipOffset, hipLength, kneeLength, false, swing_dt, swing_time);

p = gc_kinematics(p);

left_joints_traj0 = left_joints_fn(0);
right_joints_traj0 = right_joints_fn(0);
FR_config = right_joints_traj0(1:4,:);
FL_config = left_joints_traj0(1:4,:);
center_pose = [0;0;height;0;0;0];
q0 = [center_pose; FR_config; FL_config];
dq0 = zeros(nq,1);
tspan = 0:0.1:0.5;

u0 = zeros(2*p.ngc,1);
z0 = [q0;dq0;u0];

[ts, zs] = ode45(@(t,z) dynamics(t,z,p, left_joints_fn, right_joints_fn),tspan, z0);
qs_out = zs(:,1:p.NB);
showmotion(p.robot,ts',qs_out');

function zdot = dynamics(t,z,p, left_joints_fn, right_joints_fn)
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
    % ground contact dynamic
    [force, udot, fcone] = gcontact(K, D, mu, pos, vel, u);
    force6D = Fpt(force, pos);
    % move friction forces to external forces
    f_ext = cell(p.NB,1);
    for i = 1:p.NB
        f_ext{i} = zeros(6,1);
    end
    for i = 1:p.ngc
        f_ext{p.robot.gc.body(i)} = f_ext{p.robot.gc.body(i)} + force6D(:,i);
    end

    com_desired_traj = @(t) [0.05;0;0.3;0;0;0];
    actuate_tau = whole_body_control(t, z, p, com_desired_traj, left_joints_fn, right_joints_fn, mu);
    B = blkdiag(zeros(6), eye(8));
    actuate_tau = B*actuate_tau;
    ddq = FDcrb(p.robot, q, dq, actuate_tau, f_ext);

    disp(t)
    zdot = [dq; ddq; udot(:)];
end
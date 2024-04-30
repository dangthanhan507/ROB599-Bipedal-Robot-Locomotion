%{
    Final run w/ WBC just see how it goes
%}

hipOffset = 0.062;
hipLength = 0.209;
kneeLength = 0.209;

bodyWidth = 0.049*2;
bodyLength = 0.049*2;
bodyHeight = 0.05*2;

origin2hip = hipOffset+bodyWidth/2;

left_foot0 = [0; origin2hip];
right_foot0 = [0; -origin2hip];

stride = 0.10;
footsteps = [0, origin2hip;...
             stride, -origin2hip;...
             2*stride, origin2hip;...
             3*stride, -origin2hip;...
             4*stride, origin2hip;...
             5*stride, -origin2hip;...
             6*stride, origin2hip;...
             7*stride, -origin2hip;...
             8*stride, origin2hip]';

x0 = [0; 0];
height = 0.3;
single_support_duration = 0.2;
double_support_duration = 0.2;
Q = 10*eye(2);
R = eye(2);
td = 0.2;

[com_ts, com_traj] = zmp_planner(footsteps, x0, height, single_support_duration, double_support_duration, Q, R, td);

com_traj = [com_traj(1:2,:); height*ones(1, length(com_ts))];
com_traj_fn = @(t) first_order_hold(t, com_ts, com_traj);

z_max = 0.01;
[leftFootTraj, rightFootTraj] = footTrajFromFootsteps(left_foot0,  right_foot0, footsteps, z_max, single_support_duration, double_support_duration);

ts = 0:0.01:com_ts(end);
disp("Total Sim Time: " + ts(end));

%get joint angles
left_joint_pos = zeros(4, length(ts));
right_joint_pos = zeros(4, length(ts));
for i = 1:length(ts)
    [left_joint_angle, right_joint_angle] = footTraj2Joints(ts(i), com_traj_fn, leftFootTraj, rightFootTraj,...
                                                             hipOffset, hipLength, kneeLength, bodyWidth);

    left_joint_pos(:, i) = left_joint_angle;
    right_joint_pos(:, i) = right_joint_angle;
end
left_joint_traj = @(t) first_order_hold(t, ts, left_joint_pos);
right_joint_traj = @(t) first_order_hold(t, ts, right_joint_pos);

p.robot = biped_robot();
p.NB = p.robot.NB;
p.ngc = numel(p.robot.gc.body);
nq = p.NB;

p = gc_kinematics(p);

left_joints_traj0 = left_joint_traj(0);
right_joints_traj0 = right_joint_traj(0);
FR_config = right_joints_traj0(1:4,:);
FL_config = left_joints_traj0(1:4,:);
center_pose = [0;0;height;0;0;0];
q0 = [center_pose; FR_config; FL_config];
dq0 = zeros(nq,1);
tspan = ts;

u0 = zeros(2*p.ngc,1);
z0 = [q0;dq0;u0];

[ts, zs] = ode45(@(t,z) dynamics(t,z,p, left_joint_traj, right_joint_traj, com_traj_fn),tspan, z0);
qs_out = zs(:,1:p.NB);
showmotion(p.robot,ts',qs_out');

function zdot = dynamics(t,z,p, left_joints_fn, right_joints_fn, com_fn)
    K = 1e4;
    D = 2000;
    mu = 1.0;

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

    [H,C] = HandC(p.robot, q, dq);
    % move friction forces to external forces
    f_ext = cell(p.NB,1);
    for i = 1:p.NB
        f_ext{i} = zeros(6,1);
    end
    for i = 1:p.ngc
        f_ext{p.robot.gc.body(i)} = f_ext{p.robot.gc.body(i)} + force6D(:,i);
    end

    B = blkdiag(zeros(6), eye(8));
    actuate_tau = whole_body_control(t, z, p, com_fn, left_joints_fn, right_joints_fn, mu);
    actuate_tau = B*actuate_tau;

    % cancel out the torques on the body since we can't control it at the moment
    Kp_rot = 500;
    Kd_rot = 20;
    actuate_tau(4:6) = Kp_rot*(zeros(3,1) - q(4:6)) + Kd_rot*(zeros(3,1) - dq(4:6)) + C(4:6);

    ddq = FDcrb(p.robot, q, dq, actuate_tau, f_ext);

    disp(t)
    zdot = [dq; ddq; udot(:)];
end
%{
    Given:
    ======
    -> Body Force and Torque = 0.
    -> Swing-Leg Trajectory is still.

    Maintain the objectives through a QP reactive controller.
%}

%params
p.robot = biped_robot();
p.NB = p.robot.NB;
p.ngc = numel(p.robot.gc.body);

tspan = 0:0.1:2;

%initial conditions
nq = p.NB;

% joint configuration convention:
% [x,y,z,roll,pitch,yaw]
% [FR_abad, FR_hip, FR_knee]
% [FL_abad, FL_hip, FL_knee]

%start at foot to be at zero height
center_pose = [0;0;0.3;0;0;0];
FR_config = [0;deg2rad(45);deg2rad(-90); deg2rad(45)];
FL_config = [0;deg2rad(45);deg2rad(-90); deg2rad(45)];
q0 = [center_pose; FR_config; FL_config];
dq0 = zeros(nq,1);

p = gc_kinematics(p);

% not control input, this is for calculating friction forces
u0 = zeros(2*p.ngc,1);
z0 = [q0;dq0;u0];
[ts, zs] = ode45(@(t,z) dynamics(t,z,p),tspan, z0);
qs_out = zs(:,1:p.NB);

% plot the body position of robot
% plot the feet position of robot

body_pos = zeros(3, size(qs_out, 1));
left_foot_pos = zeros(3, size(qs_out, 1));
right_foot_pos = zeros(3, size(qs_out, 1));

disp(unique(p.robot.gc.body))
for i = 1:size(qs_out, 1)
    % get left and right foot positions using pvj
    [body_pos(:,i), ~, ~, ~] = get_pvj(qs_out(i,:)', zeros(p.NB,1), p, 1);
    [left_foot_pos(:,i), ~, ~, ~] = get_pvj(qs_out(i,:)', zeros(p.NB,1), p, 12);
    [right_foot_pos(:,i), ~, ~, ~] = get_pvj(qs_out(i,:)', zeros(p.NB,1), p, 17);
end

% make 2d plots time vs. x, y, z
% with legends
figure;
subplot(3,1,1);
plot(ts, body_pos(1,:), 'r');
hold on;
plot(ts, body_pos(2,:), 'g');
plot(ts, body_pos(3,:), 'b');
legend('x', 'y', 'z');
hold off


subplot(3,1,2);
plot(ts, left_foot_pos(1,:), 'r');
hold on;
plot(ts, left_foot_pos(2,:), 'g');
plot(ts, left_foot_pos(3,:), 'b');
hold off

subplot(3,1,3);
plot(ts, right_foot_pos(1,:), 'r');
hold on;
plot(ts, right_foot_pos(2,:), 'g');
plot(ts, right_foot_pos(3,:), 'b');
hold off

showmotion(p.robot,ts',qs_out');

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


    % ground contact dynamic
    [force, udot, fcone] = gcontact(K, D, mu, pos, vel, u);

    force6D = Fpt(force, pos);

    f_ext = cell(p.NB,1);
    for i = 1:p.NB
        f_ext{i} = zeros(6,1);
    end
    for i = 1:p.ngc
        f_ext{p.robot.gc.body(i)} = f_ext{p.robot.gc.body(i)} + force6D(:,i);
    end


    [H, C] = HandC(p.robot, q, dq);
    Kp = 500;
    Kd = 20;
    q_FR = q(7:10);
    q_FL = q(11:14);

    q_FR_des = [0;deg2rad(45);deg2rad(-90);deg2rad(45)];
    q_FL_des = [0;deg2rad(45);deg2rad(-90);deg2rad(45)];

    tau_FR = Kp*(q_FR_des - q_FR) - Kd*dq(7:10);
    tau_FL = Kp*(q_FL_des - q_FL) - Kd*dq(11:14);

    % com desired
    % [com, dcom, J_com, dJdq_com] = get_pvj(q, dq, p, 1);
    % com_des = [0;0;0.3];
    % tau_com = Kp*(com_des - com) - Kd*(dcom);

    tau = zeros(p.NB,1);
    % tau(1:3) = tau_com;
    tau(7:10) = tau_FR;
    tau(11:14) = tau_FL;
    tau(7:14) = H(7:14,:)*tau + C(7:14);

    disp(t)


    % q_FR_des = [0;deg2rad(45);deg2rad(-90); deg2rad(45)];
    % q_FL_des = [0;deg2rad(45);deg2rad(-90); deg2rad(45)];
    % com_desired_traj = @(t) [0;0;0.3;0;0;0];
    % left_foot_desired_traj = @(t) [q_FL_des; zeros(4,1)];
    % right_foot_desired_traj = @(t) [q_FR_des; zeros(4,1)];
    % tau = whole_body_control(t,z,p, com_desired_traj, left_foot_desired_traj, right_foot_desired_traj, mu);

    B = blkdiag(zeros(6,6), eye(8));
    tau = B*tau;
    ddq = FDcrb(p.robot, q, dq, tau, f_ext);

    zdot = [dq;ddq;udot(:)];
end
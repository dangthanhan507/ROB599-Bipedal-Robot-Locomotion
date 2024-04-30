%{
    % Given a Footstep Plan, Use ZMP to get CoM as a function of time
    % Use Footstep Plan to get (X,Y,Z) of Left and Right Foot
    % Use IK to get Joint Angles of Left and Right Foot
    % Piece it together to get Left and Right Foot angles as a function of time
%}

hipOffset = 0.062;
hipLength = 0.209;
kneeLength = 0.209;

bodyWidth = 0.049*2;
bodyLength = 0.049*2;
bodyHeight = 0.05*2;

%NOTE: footsteps start with left foot first
origin2hip = hipOffset+bodyWidth/2;

left_foot0 = [0; origin2hip];
right_foot0 = [0; -origin2hip];

stride = 0.1;
footsteps = [0, origin2hip;...
             stride, -origin2hip;...
             2*stride, origin2hip;...
             3*stride, -origin2hip;...
             4*stride, origin2hip;...
             5*stride, -origin2hip]';

x0 = [0; 0];
height = 0.3;
single_support_duration = 0.1;
double_support_duration = 0.2;
Q = 10*eye(2);
R = eye(2);
td = 0.1;


[com_ts, com_traj] = zmp_planner(footsteps, x0, height, single_support_duration, double_support_duration, Q, R, td);

% add in height
com_traj = [com_traj(1:2,:); height*ones(1, length(com_ts))];
% first-order-hold to get CoM as function
com_traj_fn = @(t) first_order_hold(t, com_ts, com_traj);

z_max = 0.1;
% get footsteps in xyz
[leftFootTraj, rightFootTraj] = footTrajFromFootsteps(left_foot0,  right_foot0, footsteps, z_max, single_support_duration, double_support_duration);

% plot footstep
% plot(footsteps(1, :), footsteps(2, :), 'ko--');

% plot 3d foot trajectories
ts = 0:0.01:com_ts(end);
n_points = length(ts);
leftFootPos = zeros(6, n_points);
rightFootPos = zeros(6, n_points);
for i = 1:n_points
    leftFootPos(:, i) = leftFootTraj(ts(i));
    rightFootPos(:, i) = rightFootTraj(ts(i));
end

figure;
plot3(leftFootPos(1, :), leftFootPos(2, :), leftFootPos(3, :), 'ro-');
hold on;
plot3(rightFootPos(1, :), rightFootPos(2, :), rightFootPos(3, :), 'bo-');

% plot CoM trajectory
com_traj_pos = zeros(6, n_points);
for i = 1:n_points
    com_traj_pos(:, i) = com_traj_fn(ts(i));
end
plot3(com_traj_pos(1, :), com_traj_pos(2, :), com_traj_pos(3, :), 'go-');
legend('Left Foot', 'Right Foot', 'CoM');
xlabel('x');
ylabel('y');
zlabel('z');
grid on;
hold off;


p.robot = biped_robot();
p.NB = p.robot.NB;
qs_out = zeros(14, length(ts));

for i = 1:n_points
    qs_out(1:3,i) = com_traj_pos(1:3, i);

    [left_joint_angles, right_joint_angles] = footTraj2Joints(ts(i), com_traj_fn, leftFootTraj, rightFootTraj,...
                                                             hipOffset, hipLength, kneeLength, bodyWidth);
    qs_out(7:10, i) = right_joint_angles;
    qs_out(11:14, i) = left_joint_angles;
end
showmotion(p.robot, ts, qs_out);
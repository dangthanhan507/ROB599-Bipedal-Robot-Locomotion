% only plot time <= 1
qs_out = zs(:,1:p.NB);
dq_out = zs(:,p.NB+1:2*p.NB);
left_joint_pos = zeros(length(ts),8);
right_joint_pos = zeros(length(ts),8);
for i = 1:length(ts)
    left_joint_pos(i,:) = left_joint_traj(ts(i));
    right_joint_pos(i,:) = right_joint_traj(ts(i));
end

%plot desired CoM
figure;
subplot(3,1,1)
plot(com_ts, com_traj(1,:), 'r');
hold on;
plot(ts, qs_out(:,1), 'b');
hold off;
title('X CoM');
xlabel('Time (s)')
ylabel('Position (m)')
legend('Desired CoM', 'Actual CoM')

subplot(3,1,2)
plot(com_ts, com_traj(2,:), 'r');
hold on;
plot(ts, qs_out(:,2), 'b');
hold off;
title('Y CoM');

subplot(3,1,3)
plot(com_ts, com_traj(3,:), 'r');
hold on;
plot(ts, qs_out(:,3), 'b');
hold off;
title('Z CoM');

% plot desired joint angles
figure;
subplot(4,2,1)
plot(ts, right_joint_pos(:,1), 'r');
hold on;
plot(ts, qs_out(:,7), 'b');
hold off;
title('Right Hip Rx');
xlabel('Time (s)')
ylabel('Angle (rad)')
legend('Desired Angle', 'Actual Angle')

subplot(4,2,3)
plot(ts, right_joint_pos(:,2), 'r');
hold on;
plot(ts, qs_out(:,8), 'b');
hold off;
title('Right Hip Ry');
xlabel('Time (s)')
ylabel('Angle (rad)')

subplot(4,2,5)
plot(ts, right_joint_pos(:,3), 'r');
hold on;
plot(ts, qs_out(:,9), 'b');
hold off;
title('Right Knee Ry');
xlabel('Time (s)')
ylabel('Angle (rad)')

subplot(4,2,7)
plot(ts, right_joint_pos(:,4), 'r');
hold on;
plot(ts, qs_out(:,10), 'b');
hold off;
title('Right Ankle Ry');
xlabel('Time (s)')
ylabel('Angle (rad)')

subplot(4,2,2)
plot(ts, left_joint_pos(:,1), 'r');
hold on;
plot(ts, qs_out(:,11), 'b');
hold off;
title('Left Hip Rx');
xlabel('Time (s)')
ylabel('Angle (rad)')
legend('Desired Angle', 'Actual Angle');

subplot(4,2,4)
plot(ts, left_joint_pos(:,2), 'r');
hold on;
plot(ts, qs_out(:,12), 'b');
hold off;
title('Left Hip Ry');
xlabel('Time (s)')
ylabel('Angle (rad)')

subplot(4,2,6)
plot(ts, left_joint_pos(:,3), 'r');
hold on;
plot(ts, qs_out(:,13), 'b');
hold off;
title('Left Knee Ry');
xlabel('Time (s)')
ylabel('Angle (rad)')

subplot(4,2,8)
plot(ts, left_joint_pos(:,4), 'r');
hold on;
plot(ts, qs_out(:,14), 'b');
hold off;
title('Left Ankle Ry');
xlabel('Time (s)')
ylabel('Angle (rad)')


% plot desired xyz of feet
left_foot_xyz = zeros(length(ts), 6);
right_foot_xyz = zeros(length(ts), 6);
for i = 1:length(ts)
    left_foot_xyz(i,:) = leftFootTraj(ts(i));
    right_foot_xyz(i,:) = rightFootTraj(ts(i));
end

% get actual xyz of feet from q
left_foot_xyz_actual = zeros(length(ts), 3);
right_foot_xyz_actual = zeros(length(ts), 3);
for i = 1:length(ts)
    [xyz_left, ~, ~, ~] = get_pvj(qs_out(i,:), dq_out(i,:), p, 17);
    [xyz_right, ~, ~, ~] = get_pvj(qs_out(i,:), dq_out(i,:), p, 12);

    left_foot_xyz_actual(i,:) = xyz_left(1:3);
    right_foot_xyz_actual(i,:) = xyz_right(1:3);
end


figure;
subplot(3,2,1)
plot(ts, left_foot_xyz(:,1), 'r');
hold on;
plot(ts, left_foot_xyz_actual(:,1), 'b');
hold off;
title('Left Foot X');
xlabel('Time (s)')
ylabel('Position (m)')
legend('Desired Position', 'Actual Position')

subplot(3,2,3)
plot(ts, left_foot_xyz(:,2), 'r');
hold on;
plot(ts, left_foot_xyz_actual(:,2), 'b');
hold off;
title('Left Foot Y');
xlabel('Time (s)')
ylabel('Position (m)')

subplot(3,2,5)
plot(ts, left_foot_xyz(:,3), 'r');
hold on;
plot(ts, left_foot_xyz_actual(:,3), 'b');
hold off;
title('Left Foot Z');
xlabel('Time (s)')
ylabel('Position (m)')

subplot(3,2,2)
plot(ts, right_foot_xyz(:,1), 'r');
hold on;
plot(ts, right_foot_xyz_actual(:,1), 'b');
hold off;
title('Right Foot X');
xlabel('Time (s)')
ylabel('Position (m)')

subplot(3,2,4)
plot(ts, right_foot_xyz(:,2), 'r');
hold on;
plot(ts, right_foot_xyz_actual(:,2), 'b');
hold off;
title('Right Foot Y');
xlabel('Time (s)')
ylabel('Position (m)')

subplot(3,2,6)
plot(ts, right_foot_xyz(:,3), 'r');
hold on;
plot(ts, right_foot_xyz_actual(:,3), 'b');
hold off;
title('Right Foot Z');
xlabel('Time (s)')
ylabel('Position (m)')

%plot rotation of com
figure;
subplot(3,1,1)
plot(ts, qs_out(:,4), 'b');
title('Roll');
xlabel('Time (s)')
ylabel('Angle (rad)')

subplot(3,1,2)
plot(ts, qs_out(:,5), 'b');
title('Pitch');
xlabel('Time (s)')
ylabel('Angle (rad)')

subplot(3,1,3)
plot(ts, qs_out(:,6), 'b');
title('Yaw');
xlabel('Time (s)')
ylabel('Angle (rad)')


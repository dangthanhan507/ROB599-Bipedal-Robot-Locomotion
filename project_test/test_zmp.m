%{
    Test ZMP Planner
%}
footsteps = [0, -0.05;...
             0.5, 0.05;...
             1, -0.05;...
             1.5, 0.05;...
             2, -0.05;...
             2.5, 0.05]';
x0 = [0; 0];
height = 0.3;
single_support_duration = 0.2;
double_support_duration = 0.1;
Q = eye(2);
R = eye(2);
td = 0.1;

[com_ts, comTraj] = zmp_planner(footsteps, x0, height, single_support_duration, double_support_duration, Q, R, td);
% plot footsteps
plot(footsteps(1, :), footsteps(2, :), 'o');
% draw line between footsteps
for i = 1:size(footsteps, 2)-1
    line([footsteps(1, i), footsteps(1, i+1)], [footsteps(2, i), footsteps(2, i+1)]);
end
hold on;
% plot CoM trajectory
plot(comTraj(1, :), comTraj(2, :), 'ro-');
hold off;
xlabel('x');
ylabel('y');
title('ZMP Planner');
xlim([-0.5 3]);
legend('Footsteps', 'CoM Trajectory');
axis equal;
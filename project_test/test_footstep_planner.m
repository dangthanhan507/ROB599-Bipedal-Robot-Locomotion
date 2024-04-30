% Test MATLAB implementation of nonlinear footstep optimization
initial_left = [0;-0.1];
initial_right = [0;0.1];
goal_pos = [2;0];
n_steps = 30;
step_span = 0.2;
feet_gap = 0.2;

[left_feet, right_feet] = nonlinear_footstep(initial_left, initial_right, goal_pos, n_steps, step_span, feet_gap);

plot(initial_left(1), initial_left(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
hold on;
plot(initial_right(1), initial_right(2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');

% plot traj
plot(left_feet(1,:), left_feet(2,:), 'ro-');
plot(right_feet(1,:), right_feet(2,:), 'bo-');

plot(goal_pos(1), goal_pos(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');

hold off
title('Nonlinear Footstep Optimization');
xlabel('x');
ylabel('y');
legend('Initial Left Foot', 'Initial Right Foot', 'Left Foot', 'Right Foot', 'Goal Position');

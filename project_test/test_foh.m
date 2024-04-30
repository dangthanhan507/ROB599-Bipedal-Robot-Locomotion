% test first_order_hold function

% make quadratic function
ts = 0:5:10;
traj_gt = [ts.^2; 2*ts];
pos = traj_gt(1, :);

% test first_order_hold
test_ts = 0:0.01:10;
traj_fn = @(t) first_order_hold(t, ts, pos);
traj = zeros(2, length(test_ts));
for i = 1:length(test_ts)
    traj(:, i) = traj_fn(test_ts(i));
end

% plot
figure;
subplot(2, 1, 1);
plot(ts, traj_gt(1, :), 'r', 'LineWidth', 2);
hold on;
plot(test_ts, traj(1, :), 'b--', 'LineWidth', 2);
xlabel('Time');
ylabel('Position');
legend('Original', 'Interpolated');
title('Position vs Time');
grid on;

subplot(2, 1, 2);
plot(ts, traj_gt(2, :), 'r', 'LineWidth', 2);
hold on;
plot(test_ts, traj(2, :), 'b--', 'LineWidth', 2);
xlabel('Time');
ylabel('Velocity');
legend('Original', 'Interpolated');
title('Velocity vs Time');
grid on;

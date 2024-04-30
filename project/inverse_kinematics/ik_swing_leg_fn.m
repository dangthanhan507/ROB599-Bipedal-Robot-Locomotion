function traj_fn = ik_swing_leg_fn(curr_xy, des_xy, height, z_max, hipOffset, hipLength, kneeLength, elbow_up, dt, t_total)
    %{
        Given x, y, l1, l2, elbow_up
        find the joint angles for the 2 link robot

        hipOffset: offset of the hip joint from the origin
        z: height of the abad joint
    %}

    x = curr_xy(1);
    y = curr_xy(2);

    x_des = des_xy(1);
    y_des = des_xy(2);

    % functions of time t \with t \in [0, 1]
    x_fn = @(t) x + (x_des - x)*t;
    y_fn = @(t) y + (y_des - y)*t;
    % get z as parabola where t=0.5 is the peak z_max starts and ends at 0
    % -b/2a = z_max
    z_fn = @(t) -4*z_max*(t-0.5)^2 + z_max;

    % ts: timeseries
    ts = 0:dt:t_total;
    n_points = length(ts);
    xs = zeros(1, n_points);
    ys = zeros(1, n_points);
    zs = zeros(1, n_points);
    for i = 1:n_points
        xs(i) = x_fn(ts(i)/t_total);
        ys(i) = y_fn(ts(i)/t_total);
        zs(i) = z_fn(ts(i)/t_total);
    end

    % get the joint angles
    abad_angles = zeros(1, n_points);
    hip_angles = zeros(1, n_points);
    knee_angles = zeros(1, n_points);
    foot_angles = zeros(1, n_points);
    for i = 1:n_points
        [abad_angle, hip_angle, knee_angle] = ik_biped_legs(xs(i), ys(i), height-zs(i), hipOffset, hipLength, kneeLength, elbow_up);
        abad_angles(i) = abad_angle;
        hip_angles(i) = hip_angle;
        knee_angles(i) = knee_angle;

        %foot angle should be such that hip_angle + knee_angle + foot_angle = 0
        foot_angles(i) = -hip_angle - knee_angle;
    end

    joint_angles = [abad_angles; hip_angles; knee_angles; foot_angles];
    % use First-Order-Hold to get trajectories
    traj_fn = @(t) first_order_hold(t, ts, joint_angles);
end
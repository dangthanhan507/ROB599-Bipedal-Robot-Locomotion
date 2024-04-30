function joint_trajs = ik_swing_leg(curr_xy, des_xy, height, z_max, hipOffset, hipLength, kneeLength, elbow_up, dt, t_total)
    %{
        Given x, y, l1, l2, elbow_up
        find the joint angles for the 2 link robot

        hipOffset: offset of the hip joint from the origin
        z: height of the abad joint
    %}
    ts = 0:dt:t_total;
    traj_fn = ik_swing_leg_fn(curr_xy, des_xy, height, z_max, hipOffset, hipLength, kneeLength, elbow_up, dt, t_total);
    n_points = length(ts);
    joint_trajs = zeros(8, n_points);
    for i = 1:n_points
        joint_trajs(:, i) = traj_fn(ts(i));
    end
end
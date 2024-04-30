function joint_trajs = ik_stance_leg(curr_xy, des_xy, height, hipOffset, hipLength, kneeLength, elbow_up, dt, t_total)
    % same computation as the swing leg
    % however, we keep z to always be 0
    ts = 0:dt:t_total;
    traj_fn = ik_stance_leg_fn(curr_xy, des_xy, height, hipOffset, hipLength, kneeLength, elbow_up, dt, t_total);
    n_points = length(ts);
    joint_trajs = zeros(8, n_points);
    for i = 1:n_points
        joint_trajs(:, i) = traj_fn(ts(i));
    end
end
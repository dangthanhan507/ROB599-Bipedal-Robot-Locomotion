function footsteps = combine_footsteps(footsteps_right,footsteps_left)
    n_steps = size(footsteps_right, 2);
    footsteps = zeros(2, 2*n_steps);
    for i = 1:n_steps
        footsteps(:, 2*i-1) = footsteps_right(:, i);
        footsteps(:, 2*i) = footsteps_left(:, i);
    end
end
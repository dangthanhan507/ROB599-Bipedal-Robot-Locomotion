function [leftFootTraj, rightFootTraj] = footTrajFromFootsteps(left_foot0, right_foot0, footsteps, z_max, single_support_duration, double_support_duration)
    %{
        Given com trajectory, and initial foot positions,
        get the (x,y,z) trajectory of the left and right foot

        ASSUME that we start moving left foot first.
        
        % wait single foot duration -> move foot
        % wait double foot duration -> stabilize
    %}


    n_steps = size(footsteps, 2);
    time = 0;
    tsteps = zeros(1, 2*n_steps);
    leftFootPos = zeros(2, 2*n_steps);
    rightFootPos = zeros(2, 2*n_steps);

    tsteps(1) = time;
    leftFootPos(:, 1) = left_foot0;
    rightFootPos(:, 1) = right_foot0;
    for i = 1:n_steps

        %start stepping
        time = time + single_support_duration;
        tsteps(2*i) = time;

        %if odd, left foot
        if mod(i, 2) == 1
            leftFootPos(1:2, 2*i) = footsteps(:, i); % step w/ left
            rightFootPos(1:2, 2*i) = rightFootPos(:, 2*i-1); % keep right
        else
            rightFootPos(1:2, 2*i) = footsteps(:, i); % step w/ right
            leftFootPos(1:2, 2*i) = leftFootPos(:, 2*i-1); % keep left
        end

        if i ~= n_steps
            %wait double support
            time = time + double_support_duration;
            tsteps(2*i+1) = time;
            leftFootPos(1:2, 2*i+1) = leftFootPos(:, 2*i); % keep left
            rightFootPos(1:2, 2*i+1) = rightFootPos(:, 2*i); % keep right
        end
    end

    leftFootTraj = @(t) getFootTraj(t, tsteps, leftFootPos, z_max);
    rightFootTraj = @(t) getFootTraj(t, tsteps, rightFootPos, z_max);
end
function footTraj = getFootTraj(t, tsteps, footPos, z_max)
    %{
        given a sequence of footsteps for one foot, give out a function of time
        that gives the (x,y,z) position of the foot
    %}

    n_segments = length(tsteps) - 1;
    % this will be piecewise function

    % find segment that t less than
    idx = find(tsteps < t, 1, 'last');
    if isempty(idx)
        idx = 1;
    end

    footPos_t = zeros(2, 1);
    footVel_t = zeros(2, 1);
    z_t = 0;
    z_vel_t = 0;
    if t < 0
        footPos_t = footPos(:, 1);
        z_t = 0;
        z_vel_t = 0;
        footVel_t = zeros(size(footPos(:, 1)));
    elseif t <= tsteps(end)
        % linear interpolation between positions
        normalized_t = (t - tsteps(idx)) / (tsteps(idx+1) - tsteps(idx));
        footPos_t(1:2,:) = footPos(:, idx) + (footPos(:, idx+1) - footPos(:, idx)) * normalized_t;

        if norm(footPos(:,idx) - footPos(:,idx+1)) < 1e-6
            % if the foot is not moving, then z should not move either
            z_t = 0;
            z_vel_t = 0;
        else
            z_t = -4*z_max*(normalized_t - 0.5)^2 + z_max;
            z_vel_t = -8*z_max*(normalized_t - 0.5);
        end
        % constant velocity between positions (finite differences)
        footVel_t = (footPos(:, idx+1) - footPos(:, idx)) / (tsteps(idx+1) - tsteps(idx));
    else
        footPos_t = footPos(:, end);
        z_t = 0;
        z_vel_t = 0;
        footVel_t = zeros(size(footPos(:, end)));
    end

    footTraj = [footPos_t; z_t; footVel_t; z_vel_t];
end
function traj = first_order_hold(t, ts, pos)
    %{
        Arguments:
        ==========
            t: time
            ts: time steps (discrete)
            pos: positions at each time step
    %}

    % given a time t, return the position and vel at that time using First-Order-Hold
    % linear interpolation between positions
    % constant velocity between positions (finite differences)

    % find the index of the time step that is less than or equal to t
    idx = find(ts <= t, 1, 'last');
    if isempty(idx)
        idx = 1;
    end

    % if the time is exactly the time step, return the position and vel at that time step
    if t < 0
        pos_t = pos(:, 1);
        vel_t = zeros(size(pos(:, 1)));
    elseif t < ts(end)
        % linear interpolation between positions
        pos_t = pos(:, idx) + (pos(:, idx+1) - pos(:, idx)) * (t - ts(idx)) / (ts(idx+1) - ts(idx));
        % constant velocity between positions (finite differences)
        vel_t = (pos(:, idx+1) - pos(:, idx)) / (ts(idx+1) - ts(idx));
    else
        pos_t = pos(:, end);
        vel_t = zeros(size(pos(:, end)));
    end

    traj = [pos_t; vel_t];
end
function [left_feet_sol, right_feet_sol] = nonlinear_footstep(initial_left, initial_right, goal_pos, n_steps, step_span, feet_gap)
    %{
        Arguments:
        =================
        initial_left: initial left foot position (2x1)
        initial_right: initial right foot position (2x1)
        goal_pos: goal position (2x1)
        n_steps: number of steps
        step_span: distance between steps
        feet_gap: distance between feet
    %}

    % create an nonlinear optimization problem
    opti = casadi.Opti();

    left_feet = opti.variable(2, n_steps);
    right_feet = opti.variable(2, n_steps);

    % footstep limits and alternate steps
    % foot_i - foot_{i-1} \in [-step_span, step_span]
    step_vector = ones(2,1)*step_span;
    for i = 2:n_steps

        if mod(i,2) == 0
            opti.subject_to(left_feet(:,i) - left_feet(:,i-1) >= -step_vector);
            opti.subject_to(left_feet(:,i) - left_feet(:,i-1) <= step_vector);

            opti.subject_to(right_feet(:,i) - right_feet(:,i-1) == zeros(2,1));
        else
            opti.subject_to(right_feet(:,i) - right_feet(:,i-1) >= -step_vector);
            opti.subject_to(right_feet(:,i) - right_feet(:,i-1) <= step_vector);

            opti.subject_to(left_feet(:,i) - left_feet(:,i-1) == zeros(2,1));
        end
    end

    % reachability
    for i = 2:n_steps
        % make sure the feet are reachable from the previous step
        opti.subject_to( (left_feet(:,i) - left_feet(:,i-1)).'*(left_feet(:,i) - left_feet(:,i-1)) <= step_span^2 )
        opti.subject_to( (right_feet(:,i) - right_feet(:,i-1)).'*(right_feet(:,i) - right_feet(:,i-1)) <= step_span^2 )

        opti.subject_to( (left_feet(:,i) - right_feet(:,i-1)).'*(left_feet(:,i) - right_feet(:,i-1)) <= step_span^2 )
        opti.subject_to( (right_feet(:,i) - left_feet(:,i-1)).'*(right_feet(:,i) - left_feet(:,i-1)) <= step_span^2 )
    end

    % enforce gap
    for i = 1:n_steps
        opti.subject_to( (left_feet(:,i) - right_feet(:,i)).'*(left_feet(:,i) - right_feet(:,i)) >= feet_gap^2 )
    end

    % boundary conditions
    opti.subject_to(left_feet(:,1) == initial_left);
    opti.subject_to(right_feet(:,1) == initial_right);

    % cost function
    cost = 0;
    for i = 1:n_steps
        cost = cost + (left_feet(:,i) - goal_pos).'*(left_feet(:,i) - goal_pos);
        cost = cost + (right_feet(:,i) - goal_pos).'*(right_feet(:,i) - goal_pos);
    end
    % use linear interpolation from feet to goal as initial gues
    left_feet_sol_guess = [linspace(initial_left(1), goal_pos(1), n_steps); linspace(initial_left(2), goal_pos(2), n_steps)];
    right_feet_sol_guess = [linspace(initial_right(1), goal_pos(1), n_steps); linspace(initial_right(2), goal_pos(2), n_steps)];

    opti.minimize(cost);
    opti.solver('ipopt')
    opti.set_initial(left_feet, left_feet_sol_guess);
    opti.set_initial(right_feet, right_feet_sol_guess);
    sol = opti.solve();

    left_feet_sol = sol.value(left_feet);
    right_feet_sol = sol.value(right_feet);
end
function [com_ts, comTraj] = zmp_planner(footsteps, x0, height, single_support_duration, double_support_duration, Q, R, td)
    %{
        Arguments:
            footsteps: 2 x n array of footsteps
            height: height of the CoM
            single_support_duration: duration of single support phase
            double_support_duration: duration of double support phase
            Q_: weight matrix for the cost function
            R_: weight matrix for the cost function
    %}
    n_steps = size(footsteps, 2);
    time = 0;
    zmpPos = zeros(2, 2*n_steps);
    tsteps = zeros(1, 2*n_steps);

    tsteps(1) = time;
    zmpPos(:, 1) = footsteps(:,  1);

    time = time + single_support_duration;
    tsteps(2) = time;
    zmpPos(:, 2) = footsteps(:, 1);

    for i = 2:n_steps
        time = time + double_support_duration;
        tsteps(2*i-1) = time;
        zmpPos(:, 2*i-1) = footsteps(:, i);

        time = time + single_support_duration;
        tsteps(2*i) = time;
        zmpPos(:, 2*i) = footsteps(:, i);
    end
    
    % return a function of time that gives desired ZMP position and vel
    % use first-order-hold technique
    % linear interpolation between positions
    % constant velocity between positions (finite differences)
    zmp_desired_fn = @(t) first_order_hold(t, tsteps, zmpPos);

    %{
        Linear Dynamics with Linear Output
        xdot = Ax + Bu
        y = Cx + Du
    %}
    grav = 9.83;
    A = zeros(4,4);
    B = zeros(4,2);
    C = zeros(2,4);

    A(1:2,3:4) = eye(2);
    B(3:4,1:2) = eye(2);
    C(1:2,1:2) = eye(2);
    D = -height/grav * eye(2);

    Q_ = Q;
    R_ = R;

    % get discrete versions of A,B,C,D
    sys = ss(A, B, C, D);
    sysd = c2d(sys, td);
    [Ad, Bd, Cd, Dd] = ssdata(sysd);
    %{
        Solve Finite-time Discrete TV-LQR
        x[k+1] = Ad*x[k] + Bd*u[k]
        J = g(x,u)
    %}
    % solve QP
    lqr_times = 0:td:time;
    n_lqr_steps = size(lqr_times, 2);
    opti = casadi.Opti('conic');
    x = opti.variable(4, n_lqr_steps);
    u = opti.variable(2, n_lqr_steps-1);

    disp("LQR Horizon Length: " + size(lqr_times));


    yref_fn = @(t) [eye(2), zeros(2,2)] * zmp_desired_fn(t);
    yref_t0 = yref_fn(0);
    yref_tf = yref_fn(time);
    xbar = x - [yref_tf; zeros(2,1)]; % x - xref
    ybar = @(t) yref_fn(t) - yref_tf;
    

    Q1 = Cd' * Q_ * Cd;
    q2 = @(t) -2 * Cd' * Q_ * ybar(t);
    q3 = @(t) ybar(t)' * Q_ * ybar(t);

    R1 = R_ * Dd' * Q_ * Dd;
    r2 = @(t) -2 * R_ * Dd' * Q_ * ybar(t);
    N = Cd' * Q_ * Dd; 


    % setup costs

    cost = 0;
    for i = 1:n_lqr_steps-1
        cost = cost + xbar(:,i)' * Q1 * xbar(:,i)...
            + xbar(:,i)' * q2(lqr_times(i)) + q3(lqr_times(i));
        cost = cost + u(:,i)' * R1 * u(:,i) + u(:,i)' * r2(lqr_times(i));
        cost = cost + 2 * xbar(:,i)' * N * u(:,i);
    end
    cost = cost + xbar(:,n_lqr_steps)' * Q1 * xbar(:,n_lqr_steps)...
        + xbar(:,n_lqr_steps)' * q2(lqr_times(n_lqr_steps)) + q3(lqr_times(n_lqr_steps));
    
    % setup constraints
    for i = 1:n_lqr_steps-1
        opti.subject_to(x(:,i+1) == Ad * x(:,i) + Bd * u(:,i));
    end
    opti.subject_to(Cd*x(:,n_lqr_steps) == yref_tf);

    % add large bounds for u
    for i = 1:n_lqr_steps-1
        opti.subject_to(u(:,i) >= -100);
        opti.subject_to(u(:,i) <= 100);
    end

    % add boundary conditions
    opti.subject_to(x(:,1) == [x0;zeros(2,1)]);

    disp('Solving LQR')
    % solve
    opti.minimize(cost);
    solver_options = struct;

    solver_options.printLevel = 'none';
    solver_options.error_on_fail = false;
    opti.solver('qpoases', solver_options);
    sol = opti.solve();

    
    x_opt = sol.value(x);
    comTraj = x_opt;
    com_ts = lqr_times;
end
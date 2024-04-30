function tau_actuate = whole_body_control(t,z,p, com_desired_traj, left_foot_desired_traj, right_foot_desired_traj, friction_mu)

    % breaking down state vector into q and dq
    idx = 0;
    q = z(idx+(1:p.NB));
    idx = idx + p.NB;
    dq = z(idx+(1:p.NB));

    com_desired_state = com_desired_traj(t);
    com_desired = com_desired_state(1:3);
    comdot_desired = com_desired_state(4:6);

    left_foot_desired_state = left_foot_desired_traj(t);
    left_foot_desired = left_foot_desired_state(1:4);
    left_foot_desired_dot = left_foot_desired_state(5:8);

    right_foot_desired_state = right_foot_desired_traj(t);
    right_foot_desired = right_foot_desired_state(1:4);
    right_foot_desired_dot = right_foot_desired_state(5:8);

    % xddot = J(q)*qddot + dJ(q,qdot)*qdot % J = jacobian of com w.r.t q
    % x = com, dx = comdot, J = jac of com, dJdq = Jdot * qdot
    [~, dx_com, J_com, dJdq_com] = get_pvj(q, dq, p, 1);
    J_com = J_com(1:3, :); % only x,y,z components
    dJdq_com = dJdq_com(1:3, :); % only x,y,z components

    x_com = q(1:3);
    com = q(1:6);
    comdot = dq(1:6);


    % get comddot_ref = Kp*(com_desired - com) + Kd*(comdot_desired - comdot)
    Kp_com = 500;
    Kd_com = 20;
    comddot_ref = Kp_com*(com_desired - x_com) + Kd_com*(comdot_desired - dx_com);

    % cost_com = norm(comddot_ref - comddot)^2
    % comddot = J_com*qddot + dJdq_com*dq
    % NOTE: qddot will be decision variable


    [H, C] = HandC(p.robot, q, dq);

    [x_right, ~, Jcontact_right, dJdq_left] = get_pvj(q, dq, p, 12);
    [x_left, ~, Jcontact_left, dJdq_right] = get_pvj(q, dq, p, 15);

    %only x,y,z (tangential, normal) coordinates considered.
    Jcontact_right = Jcontact_right(1:3, :);
    Jcontact_left = Jcontact_left(1:3, :);
    dJdq_right = dJdq_right(1:3, :);
    dJdq_left = dJdq_left(1:3, :);

    FR_config = q(7:10);
    FR_configdot = dq(7:10);
    
    FL_config = q(11:14);
    FL_configdot = dq(11:14);

    % left_foot(4) + left_foot(2) + left_foot(3) + com(5) = 0
    left_foot_desired(4) = -FL_config(2) - FL_config(3) + com(5);
    left_foot_desired_dot(4) = -FL_configdot(2) - FL_configdot(3) + comdot(5);

    % right_foot(4) + right_foot(2) + right_foot(3) + com(5) = 0
    right_foot_desired(4) = -FR_config(2) - FR_config(3) + com(5);
    right_foot_desired_dot(4) = -FR_configdot(2) - FR_configdot(3) + comdot(5);

    Kp_foot = 500;
    Kd_foot = 20;
    left_footddot_ref = Kp_foot*(left_foot_desired - FL_config) + Kd_foot*(left_foot_desired_dot - FL_configdot);
    right_footddot_ref = Kp_foot*(right_foot_desired - FR_config) + Kd_foot*(right_foot_desired_dot - FR_configdot);



    % cost_left_foot = norm(left_footddot_ref - left_footddot)^2
    % left_footddot = Jcontact_left*qddot + dJdq_left
    % cost_right_foot = norm(right_footddot_ref - right_footddot)^2
    % right_footddot = Jcontact_right*qddot + dJdq_right

    % tau_ext = Jcontact_right' * f_right + Jcontact_left' * f_left
    % lambda = [f_right; f_left]
    % NOTE: lambda will be decision variable

    % H(q)*qddot + C*qdot = tau + tau_ext
    % this is linear equality constraint

    % we want lambda to follow the friction cone constraints given friction_mu
    % use polyhedral approximation
    cone_sides = 10;
    A_cone = zeros(cone_sides, 3);
    b_cone = zeros(cone_sides, 1);
    for i = 1:cone_sides
        theta = 2*pi*i/cone_sides;
        A_cone(i, :) = [cos(theta)/friction_mu, sin(theta)/friction_mu, 1];
        b_cone(i) = 0;
    end
    % [A_cone, Acone] * lambda >= [b_cone; b_cone]

    %last constraint is an engineering constraint
    %Jqddot + dJdq = -alpha*J*qdot + eta
    % make sure qddot is inversely proportional to qdot
    % alpha is a constant
    % eta is a decision variable (slack) to make sure constraint is feasible.
    % regularize eta in the cost function

    %{
        Solve QP using CasADi

        min norm(comddot_ref - comddot)^2 + norm(left_footddot_ref - left_footddot)^2 + norm(right_footddot_ref - right_footddot)^2 + norm(eta)^2
        s.t.
        H(q)*qddot + C*qdot = tau + tau_ext
        [A_cone, Acone] * lambda <= [b_cone; b_cone]
        Jqddot + dJdq = -alpha*J*qdot + eta
        eta_min <= eta <= eta_max
        tau_ext = [Jcontact_right', Jcontact_left'] * lambda

        decision variables: qddot, tau, lambda, eta
    %}

    opti = casadi.Opti('conic');

    %deicsion variables
    qddot = opti.variable(p.NB);
    lambda = opti.variable(3*2);
    tau = opti.variable(p.NB);
    tau_min = -100;
    tau_max = 100;

    % cost function
    cost = 0;
    cost = cost + sum((comddot_ref - (J_com*qddot + dJdq_com)).^2); % com PD tracking
    cost = cost + sum((left_footddot_ref - qddot(11:14)).^2); % left foot PD tracking
    cost = cost + sum((right_footddot_ref - qddot(7:10)).^2); % right foot PD tracking
    cost = cost + sum(lambda.^2); % lambda regularization

    % add constraints
    B = blkdiag(zeros(6,6), eye(8));

    % opti.subject_to(H*qddot + C == B*tau); % dynamics
    friction_expr = zeros(14, 1);
    if x_left(3) <= 1e-3
        % only left foot is in contact
        friction_expr = friction_expr + Jcontact_left'*lambda(1:3);

        % make sure foot actuation is within friction cone
        % J'*f = tau
        % f = inv(J*J')*J*tau
    elseif x_right(3) <= 1e-3
        % only right foot is in contact
        friction_expr = friction_expr + Jcontact_right'*lambda(4:6);
    end
    opti.subject_to(H*qddot + C == B*tau + friction_expr); % dynamics
    opti.subject_to(blkdiag(A_cone, A_cone)*lambda >= [b_cone; b_cone]); % friction cone

    % alpha = 0.1;
    % eta_min = -10.0;
    % eta_max = 10.0;
    % if x_left(3) <= 1e-3
    %     eta_left = opti.variable(3);
    %     cost = cost + sum(eta_left.^2); % eta_left regularization
    %     opti.subject_to(eta_left <= eta_max); % slack variable bounds
    %     opti.subject_to(eta_left >= eta_min); % slack variable bounds
    %     opti.subject_to(Jcontact_left*qddot + dJdq_left + alpha*Jcontact_left*dq == eta_left); % engineering constraint
    % end
    % if x_right(3) <= 1e-3
    %     eta_right = opti.variable(3);
    %     cost = cost + sum(eta_right.^2); % eta_right regularization
    %     opti.subject_to(eta_right <= eta_max); % slack variable bounds
    %     opti.subject_to(eta_right >= eta_min); % slack variable bounds
    %     opti.subject_to(Jcontact_right*qddot + dJdq_right + alpha*Jcontact_right*dq == eta_right); % engineering constraint
    % end


    opti.subject_to(tau <= tau_max); % torque bounds
    opti.subject_to(tau >= tau_min); % torque bounds
    opti.minimize(cost);

    % qpoases solver
    % no prints for qpoases
    solver_options = struct;

    solver_options.printLevel = 'none';
    solver_options.error_on_fail = false;
    opti.solver('qpoases', solver_options);
    sol = opti.solve();

    tau_actuate = sol.value(tau);

    Kp = 500;
    Kd = 20;
    tau_actuate(10) = Kp*(right_foot_desired(4) - FR_config(4)) + Kd*(right_foot_desired_dot(4) - FR_configdot(4));
    tau_actuate(14) = Kp*(left_foot_desired(4) - FL_config(4)) + Kd*(left_foot_desired_dot(4) - FL_configdot(4));

    tau_actuate = B*tau_actuate;
end
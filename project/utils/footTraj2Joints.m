function [joint_left, joint_right] = footTraj2Joints(t, com_traj_fn, left_traj_fn, right_traj_fn, hipOffset, hipLength, kneeLength, bodyWidth)
    %{
    %}

    % get relative position
    com = com_traj_fn(t);
    com = com(1:3);
    left = left_traj_fn(t);
    left = left(1:3);
    right = right_traj_fn(t);
    right = right(1:3);


    rel_left = left - (com + [0; bodyWidth/2; 0]);
    rel_left(3) = -rel_left(3);
    rel_right = right - com + [0; bodyWidth/2 + 2*hipOffset; 0];
    rel_right(3) = -rel_right(3);

    % disp(rel_left);
    %reverse direction of z for ik

    % get joint angles
    [abad_angle, hip_angle, knee_angle] = ik_biped_legs(rel_left(1), rel_left(2), rel_left(3), hipOffset, hipLength, kneeLength, true);
    foot_angle = -hip_angle - knee_angle; % foot angle should be such that hip_angle + knee_angle + foot_angle = 0
    joint_left = [abad_angle; hip_angle; knee_angle; foot_angle];

    % disp(abad_angle);
    % disp(hip_angle);
    % disp(knee_angle);
    % input('')

    [abad_angle, hip_angle, knee_angle] = ik_biped_legs(rel_right(1), rel_right(2), rel_right(3), hipOffset, hipLength, kneeLength, true);
    foot_angle = -hip_angle - knee_angle; % foot angle should be such that hip_angle + knee_angle + foot_angle = 0
    joint_right = [abad_angle; hip_angle; knee_angle; foot_angle];

    
end
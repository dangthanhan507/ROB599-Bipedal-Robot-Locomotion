%{ 
    visualize double leg ik
%}

hipOffset = 0.062;
hipLength = 0.209;
kneeLength = 0.209;

bodyWidth = 0.049*2;
bodyLength = 0.049*2;
bodyHeight = 0.05*2;

height = 0.3;

curr_left_xy = [0.0, hipOffset]';
curr_right_xy = [0.0, hipOffset]';

des_left_xy = [-0.1, hipOffset]';
des_right_xy = [-0.1, hipOffset]';

z_max = 0.1;

left_joints = ik_stance_leg(curr_left_xy, des_left_xy, height, hipOffset, hipLength, kneeLength, true, 0.01, 1);
right_joints = ik_swing_leg(curr_right_xy, des_right_xy, height, z_max, hipOffset, hipLength, kneeLength, false, 0.01, 1);

figure;
for i=1:size(left_joints, 2)
    clf;
    cube = drawCube([0;0;height], bodyWidth, true);
    hold on;    
    abad_angle_left = left_joints(1, i);
    hip_angle_left = left_joints(2, i);
    knee_angle_left = left_joints(3, i);
    
    abad_angle_right = right_joints(1, i);
    hip_angle_right = right_joints(2, i);
    knee_angle_right = right_joints(3, i);

    [location_abad_left, location_hip_left, location_knee_left, location_foot_left] = fk_biped_leg(abad_angle_left - pi, hip_angle_left, knee_angle_left, hipOffset, hipLength, kneeLength);
    [location_abad_right, location_hip_right, location_knee_right, location_foot_right] = fk_biped_leg(abad_angle_right, hip_angle_right - pi, knee_angle_right, hipOffset, hipLength, kneeLength);

    Tleft = [0;bodyWidth/2;height];
    Tright = [0;-bodyWidth/2;height];
    
    location_abad_left = location_abad_left + Tleft;
    location_hip_left = location_hip_left + Tleft;
    location_knee_left = location_knee_left + Tleft;
    location_foot_left = location_foot_left + Tleft;
    
    location_abad_right = location_abad_right + Tright;
    location_hip_right = location_hip_right + Tright;
    location_knee_right = location_knee_right + Tright;
    location_foot_right = location_foot_right + Tright;

    left_leg = drawLeg(location_abad_left, location_hip_left, location_knee_left, location_foot_left, false);
    right_leg = drawLeg(location_abad_right, location_hip_right, location_knee_right, location_foot_right, false);
    
    xlabel('x');
    ylabel('y');
    zlabel('z');
    title('Biped robot inverse kinematics');
    axis equal;
    xlim([-0.2, 0.2]);
    ylim([-0.2, 0.2]);
    zlim([-0.05, 0.5]);
    drawnow;
    pause(0.01);
    
end






function [location_left, location_right] = ik_biped(left_xyz, right_xyz, height, hipOffset, hipLength, kneeLength)
    left_x = left_xyz(1);
    left_y = left_xyz(2);
    left_z = left_xyz(3);

    right_x = right_xyz(1);
    right_y = right_xyz(2);
    right_z = right_xyz(3);

    % get the joint angles
    [abad_angle_left, hip_angle_left, knee_angle_left] = ik_biped_legs(left_x, left_y, height-left_z, hipOffset, hipLength, kneeLength, true);
    [abad_angle_right, hip_angle_right, knee_angle_right] = ik_biped_legs(right_x, right_y, height-right_z, hipOffset, hipLength, kneeLength, false);

    [location_abad_left, location_hip_left, location_knee_left, location_foot_left] = fk_biped_leg(abad_angle_left - pi, hip_angle_left, knee_angle_left, hipOffset, hipLength, kneeLength);
    [location_abad_right, location_hip_right, location_knee_right, location_foot_right] = fk_biped_leg(abad_angle_right, hip_angle_right - pi, knee_angle_right, hipOffset, hipLength, kneeLength);

    location_left = [location_abad_left, location_hip_left, location_knee_left, location_foot_left];
    location_right = [location_abad_right, location_hip_right, location_knee_right, location_foot_right];

end
function nothing = drawCube(center, side, hold_bool)
    nothing = 0;
    x = center(1);
    y = center(2);
    z = center(3);
    s = side;
    % draw the bottom square
    plot3([x-s/2, x+s/2], [y-s/2, y-s/2], [z-s/2, z-s/2], 'k');
    if hold_bool
        hold on;
    end
    plot3([x-s/2, x+s/2], [y+s/2, y+s/2], [z-s/2, z-s/2], 'k');
    plot3([x-s/2, x-s/2], [y-s/2, y+s/2], [z-s/2, z-s/2], 'k');
    plot3([x+s/2, x+s/2], [y-s/2, y+s/2], [z-s/2, z-s/2], 'k');
    % draw the top square
    plot3([x-s/2, x+s/2], [y-s/2, y-s/2], [z+s/2, z+s/2], 'k');
    plot3([x-s/2, x+s/2], [y+s/2, y+s/2], [z+s/2, z+s/2], 'k');
    plot3([x-s/2, x-s/2], [y-s/2, y+s/2], [z+s/2, z+s/2], 'k');
    plot3([x+s/2, x+s/2], [y-s/2, y+s/2], [z+s/2, z+s/2], 'k');
    % draw the vertical lines
    plot3([x-s/2, x-s/2], [y-s/2, y-s/2], [z-s/2, z+s/2], 'k');
    plot3([x+s/2, x+s/2], [y-s/2, y-s/2], [z-s/2, z+s/2], 'k');
    plot3([x-s/2, x-s/2], [y+s/2, y+s/2], [z-s/2, z+s/2], 'k');
    plot3([x+s/2, x+s/2], [y+s/2, y+s/2], [z-s/2, z+s/2], 'k');
    
end
function nothing = drawLeg(location_abad, location_hip, location_knee, location_foot, hold_bool)
    nothing = 0;

    % plot the first leg
    plot3([location_abad(1), location_hip(1)], [location_abad(2), location_hip(2)], [location_abad(3), location_hip(3)], 'r', 'LineWidth', 2);
    if hold_bool
        hold on;
    end
    % plot the second leg
    plot3([location_hip(1), location_knee(1)], [location_hip(2), location_knee(2)], [location_hip(3), location_knee(3)], 'b', 'LineWidth', 2);
    plot3([location_knee(1), location_foot(1)], [location_knee(2), location_foot(2)], [location_knee(3), location_foot(3)], 'g', 'LineWidth', 2);

    % plot the joints
    plot3(location_abad(1), location_abad(2), location_abad(3), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    plot3(location_hip(1), location_hip(2), location_hip(3), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    plot3(location_knee(1), location_knee(2), location_knee(3), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    plot3(location_foot(1), location_foot(2), location_foot(3), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
end

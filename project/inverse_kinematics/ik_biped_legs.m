function [abad_angle, hip_angle, knee_angle] = ik_biped_legs(x_des,y_des, z, hip_offset, hipLength, kneeLength, elbow_up)
    %{
        Given x, y, l1, l2, elbow_up
        find the joint angles for the 2 link robot

        hip_offset: offset of the hip joint from the origin
        z: height of the abad joint
    %}
    % abad joint works along y-direction
    abad_angle = atan2(y_des, z) - atan2(hip_offset, sqrt(y_des^2 + z^2));

    h = sqrt(y_des^2 + z^2 - hip_offset^2); % distance from hip to foot
    [hip_angle, knee_angle] = ik_2link(h, x_des, hipLength, kneeLength, elbow_up);
end
function [theta1, theta2] = ik_2link(x, y, l1, l2, elbow_up)
    % elbow_up is a bool
    if elbow_up
        theta2 = acos((x^2 + y^2 - l1^2 - l2^2) / (2*l1*l2));
        theta1 = atan2(y, x) - atan2(l2*sin(theta2), l1 + l2*cos(theta2));
    else
        theta2 = -acos((x^2 + y^2 - l1^2 - l2^2) / (2*l1*l2));
        theta1 = atan2(y, x) - atan2(l2*sin(theta2), l1 + l2*cos(theta2));
    end
end
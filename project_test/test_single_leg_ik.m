%{
    visualize example ik
    it's a 3d plot this time
%}

hipOffset = 0.062;
hipLength = 0.209;
kneeLength = 0.209;

z = 0.3;
x_des = 0.1;
y_des = hipOffset+0.1;

[abad_angle, hip_angle, knee_angle] = ik_biped_legs(x_des, y_des, z, hipOffset, hipLength, kneeLength, true);

[location_abad, location_hip, location_knee, location_foot] = fk_biped_leg(abad_angle - pi, hip_angle, knee_angle, hipOffset, hipLength, kneeLength);

location_abad = location_abad + [0;0;z];
location_hip = location_hip + [0;0;z];
location_knee = location_knee + [0;0;z];
location_foot = location_foot + [0;0;z];

% visualize the robot
% draw circles at each joint location
% draw lines between the joints
% draw goal location
figure;
% plot the first leg
plot3([location_abad(1), location_hip(1)], [location_abad(2), location_hip(2)], [location_abad(3), location_hip(3)], 'r', 'LineWidth', 2);
hold on;
% plot the second leg
plot3([location_hip(1), location_knee(1)], [location_hip(2), location_knee(2)], [location_hip(3), location_knee(3)], 'b', 'LineWidth', 2);
plot3([location_knee(1), location_foot(1)], [location_knee(2), location_foot(2)], [location_knee(3), location_foot(3)], 'g', 'LineWidth', 2);

% plot the joints
plot3(location_abad(1), location_abad(2), location_abad(3), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
plot3(location_hip(1), location_hip(2), location_hip(3), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
plot3(location_knee(1), location_knee(2), location_knee(3), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
plot3(location_foot(1), location_foot(2), location_foot(3), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
% add labels
text(location_abad(1), location_abad(2), location_abad(3), 'abad');
text(location_hip(1), location_hip(2), location_hip(3), 'hip');
text(location_knee(1), location_knee(2), location_knee(3), 'knee');
text(location_foot(1), location_foot(2), location_foot(3), 'foot');

% plot the goal
plot3(x_des, y_des, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');

hold off;

xlabel('x');
ylabel('y');
zlabel('z');
title('Biped robot inverse kinematics');
axis equal;
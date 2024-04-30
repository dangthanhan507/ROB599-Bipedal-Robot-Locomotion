%{
    Same test as test_double_leg_ik.m
    but now using showmotion to make sure we're consistent
%}

p.robot = biped_robot();
p.NB = p.robot.NB;

hipOffset = 0.062;
hipLength = 0.209;
kneeLength = 0.209;

bodyWidth = 0.049*2;
bodyLength = 0.049*2;
bodyHeight = 0.05*2;

curr_left_xy = [0.0, hipOffset]';
curr_right_xy = [0.0, hipOffset]';

des_left_xy = [0.05, hipOffset]';
des_right_xy = [0.05, hipOffset]';

height = 0.3;
z_max = 0.1;

swing_time = 1.0;
swing_dt = 0.01;
% left_joints = ik_stance_leg(curr_left_xy, des_left_xy, height, hipOffset, hipLength, kneeLength, true, 0.01, 1);
left_joints = ik_swing_leg(curr_left_xy, des_left_xy, height, z_max, hipOffset, hipLength, kneeLength, true, swing_dt, swing_time);
right_joints = ik_stance_leg(curr_right_xy, des_right_xy, height, hipOffset, hipLength, kneeLength, false, swing_dt, swing_time);

disp(left_joints(1:4,1))

left_joints = -left_joints;
% center position
center = [0;0;height+0.1];

ts = linspace(0,swing_time, size(left_joints, 2));
qs_out = zeros(14, length(ts));

qs_out(1:3,:) = repmat(center, 1, length(ts));
qs_out(7:10,:) = right_joints(1:4,:);
qs_out(11:14,:) = left_joints(1:4,:);


showmotion(p.robot,ts,qs_out);
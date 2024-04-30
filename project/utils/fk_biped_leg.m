function [abad, hip, knee, foot] = fk_biped_leg(abad_angle, hip_angle, knee_angle, hipOffset, hipLength, kneeLength)

    l1 = hipOffset;
    l2 = hipLength;
    l3 = kneeLength;
    % d = [rz, tz, tx, rx] % denavit hartenberg parameters
    d_abad2hip = [pi/2 + abad_angle, 0, l1, pi/2];
    d_hip2hip  = [pi/2, 0, 0, pi/2];
    d_hip2knee = [pi/2 + hip_angle, 0, l2, 0];
    d_knee2foot = [knee_angle, 0, l3, 0];

    T_world2abad = eye(4);
    T_world2abad(1:3,1:3) = ry(pi/2);
    T_abad2hip = dh2Homog(d_abad2hip);
    T_hip2hip = dh2Homog(d_hip2hip);
    T_hip2knee = dh2Homog(d_hip2knee);
    T_knee2foot = dh2Homog(d_knee2foot);

    % get FK for foot
    T_abad2foot = T_world2abad * T_abad2hip * T_hip2hip * T_hip2knee * T_knee2foot;
    foot = T_abad2foot(1:3,4);

    % get FK for knee
    T_abad2knee = T_world2abad * T_abad2hip * T_hip2hip * T_hip2knee;
    knee = T_abad2knee(1:3,4);

    % get FK for hip
    T_abad2hip = T_world2abad * T_abad2hip * T_hip2hip;
    hip = T_abad2hip(1:3,4);

    % get FK for abad
    abad = [0, 0, 0]';
end
function H = dh2Homog(d)
    %{
        d = [rz, tz, tx, rx] % denavit hartenberg parameters
    %}
    rZ = eye(4);
    tZ = eye(4);
    tX = eye(4);
    rX = eye(4);

    rZ(1:3,1:3) = rz(d(1));
    tZ(1:3,4) = [0; 0; d(2)];
    tX(1:3,4) = [d(3); 0; 0];
    rX(1:3,1:3) = rx(d(4));
    H = rZ * tZ * tX * rX;
end
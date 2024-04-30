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
function robot = biped_robot()
    %{
        Naming Conventions:
        FR = Front Right
        FL = Front Left
    %}

    % Robot Parameters
    %%%%%%%%%%%%%%%%%%%%%
    bodyLength = 0.049 * 2;
    bodyWidth = 0.049 * 2;
    bodyHeight = 0.05 * 2;
    hipLinkLength = 0.209;
    kneeLinkLength = 0.209;

    hipLocation = [0 0.062 0];
    kneeLocation = [0 0 -hipLinkLength];
    footLocation = [0 0 -kneeLinkLength];

    
    bodyRotationalInertia = [1, 0, 0;...
                             0, 1, 0;...
                             0, 0, 1];

    abadRotationalInertia = [1, 0, 0;...
                             0, 1, 0;...
                             0, 0, 1];

    hipRotationalInertia = [1, 0, 0;...
                            0, 1, 0;...
                             0, 0, 1];

    kneeRotationalInertia = [1, 0, 0;...
                            0, 1, 0;...
                             0, 0, 1];
    kneeRotationalInertia = ry(pi/2) * kneeRotationalInertia * ry(pi/2)';

    footRotationalInertia = [1, 0, 0;...
                            0, 1, 0;...
                             0, 0, 1];

    bodyMass = 5.0;
    abadMass = 0.54*1e-3;
    hipMass = 0.634*1e-3;
    kneeMass = 0.064;
    footMass = 0.064;

    bodyCOM = [0 0 0]';
    abadCOM = [0, 0.036, 0]';
    hipCOM = [0, 0.016, -0.02]';
    kneeCOM = [0, 0, -0.061]';
    %%%%%%%%%%%%%%%%%%%%%

    % number of bodies
    n = 9;
    robot.NB = n;    

    % number on here is body number
    %% Xtree of biped
    %     \ | /
    %       o 1   body
    %    / /                 
    %   FR FL
    %   o2 o6
    %   |  | 
    %   o3 o7
    %   |  | 
    %   o4 o8
    %   |  |
    %   o5 o9

    % DEFINE JOINT STRUCTURE
    %%%%%%%%%%%%%%%%%%%%%%%%%

    %joint parents in kinematic tree
    robot.parent = [0 1 2 3 4 1 6 7 8];

    %joint types
    %hips (Left Right)
    robot.jtype{2} = 'Rx'; robot.jtype{6} = 'Rx';
    robot.jtype{3} = 'Ry'; robot.jtype{7} = 'Ry';

    %knees
    robot.jtype{4} = 'Ry'; robot.jtype{8} = 'Ry';

    %ankles
    robot.jtype{5} = 'Ry'; robot.jtype{9} = 'Ry';

    % define locations of joints
    robot.Xtree{1} = eye(6);

    % hip location
    robot.Xtree{2} = xlt([ 0, -bodyWidth, 0]/2); 
    robot.Xtree{6} = xlt([ 0,  bodyWidth, 0]/2);

    % hip location w/ rotation
    robot.Xtree{3} = rotz(pi) * xlt([ hipLocation(1), -hipLocation(2), 0]); 
    robot.Xtree{7} = rotz(pi) * xlt([ hipLocation(1),  hipLocation(2), 0]);

    % knee location
    robot.Xtree{4} = xlt(kneeLocation);
    robot.Xtree{8} = xlt(kneeLocation);

    % foot location
    robot.Xtree{5} = xlt(footLocation);
    robot.Xtree{9} = xlt(footLocation);

    % Spatial Inertial Parameters
    robot.I{1} = mcI(bodyMass, bodyCOM, bodyRotationalInertia);

    robot.I{2} = mcI(abadMass,[abadCOM(1) -abadCOM(2) abadCOM(3)], flipInertialY(abadRotationalInertia));
    robot.I{6} = mcI(abadMass, abadCOM, abadRotationalInertia);

    robot.I{3} = mcI(hipMass,[hipCOM(1) -hipCOM(2) hipCOM(3)], flipInertialY(hipRotationalInertia));
    robot.I{7} = mcI(hipMass, hipCOM, hipRotationalInertia);

    robot.I{4} = mcI(kneeMass,[kneeCOM(1) -kneeCOM(2) kneeCOM(3)], flipInertialY(kneeRotationalInertia));
    robot.I{8} = mcI(kneeMass, kneeCOM, kneeRotationalInertia);

    robot.I{5} = mcI(footMass, [0 0 0], flipInertialY(footRotationalInertia));
    robot.I{9} = mcI(footMass, [0 0 0], footRotationalInertia);
    %%%%%%%%%%%%%%%%%%%%%%%%%

    % Define Appearance
    %%%%%%%%%%%%%%%%%%%%%%%%%
    robot.appearance.base = { 'tiles', [-50 50; -50 50; 0 0], 0.5 }; % this is the ground
    robot.appearance.body{1} = {
        'box', [ bodyLength/2  bodyWidth/2 -bodyHeight/2; -bodyLength/2 -bodyWidth/2  bodyHeight/2], ...
    };

    %draw hip joint (first part)
    robot.appearance.body{2} = { 'cyl', [ 0 0 0; 0 -0.05 0], 0.03 };
    robot.appearance.body{6} = { 'cyl', [ 0 0 0; 0  0.05 0], 0.03 };

    % draw hip joint (second part) + hip->knee linkage
    robot.appearance.body{3} = { 'cyl', [ 0 -0.016 0; 0 0.016 0], 0.025, 'box', [-0.015, -0.01, 0; 0.015, 0.01, - hipLinkLength]};
    robot.appearance.body{7} = { 'cyl', [ 0 -0.016 0; 0 0.016 0], 0.025, 'box', [-0.015, -0.01, 0; 0.015, 0.01, - hipLinkLength] };

    % draw knee joint + knee->foot linkage
    % sphere_botleft = [-0.1, -0.05, 0]';
    % sphere_botright = [-0.1, 0.05, 0]';
    % sphere_topleft = [0.1, -0.05, 0]';
    % sphere_topright = [0.1, 0.05, 0]';
    sphere_bot = [-0.1, 0, 0]';
    sphere_top = [0.1, 0, 0]';


    robot.appearance.body{4} = { 'cyl', [ 0 -0.016 0; 0 0.016 0], 0.025,...
                                 'box', [-0.015, -0.01, 0; 0.015, 0.01, - kneeLinkLength]};

    robot.appearance.body{8} = { 'cyl', [ 0 -0.016 0; 0 0.016 0], 0.025,...
                                 'box', [-0.015, -0.01, 0; 0.015, 0.01, - kneeLinkLength]};

    % draw foot
    % robot.appearance.body{5} = {'sphere',[0,0,0], 0.02,...
    %                             'sphere', sphere_botleft, 0.02,...
    %                             'sphere', sphere_botright, 0.02,...
    %                             'sphere', sphere_topleft, 0.02,...
    %                             'sphere', sphere_topright, 0.02};
    % robot.appearance.body{9} = {'sphere',[0,0,0], 0.02,...
    %                             'sphere', sphere_botleft, 0.02,...
    %                             'sphere', sphere_botright, 0.02,...
    %                             'sphere', sphere_topleft, 0.02,...
    %                             'sphere', sphere_topright, 0.02};
    robot.appearance.body{5} = {'sphere', [0,0,0], 0.02,...
                                'sphere', sphere_bot, 0.02,...
                                'sphere', sphere_top, 0.02};
    robot.appearance.body{9} = {'sphere', [0,0,0], 0.02,...
                                'sphere', sphere_bot, 0.02,...
                                'sphere', sphere_top, 0.02};
    %%%%%%%%%%%%%%%%%%%%%%%%%

    %% Camera settings
    robot.camera.body = 1;
    robot.camera.direction = [5, 20, 12];
    robot.camera.locus = [0 0];

    % GROUND CONTACT SETTINGS
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % define contacts around the body
    robot.gc.point(:,1) = [ 0; 0; 0]; % center of body
    robot.gc.point(:,2) = [ bodyLength/2 + 0.05;-bodyWidth/2 - 0.06; -bodyHeight/2];
    robot.gc.point(:,3) = [ bodyLength/2 + 0.05;-bodyWidth/2 - 0.06;  bodyHeight/2];
    robot.gc.point(:,4) = [ bodyLength/2 + 0.05; bodyWidth/2 + 0.06; -bodyHeight/2];
    robot.gc.point(:,5) = [ bodyLength/2 + 0.05; bodyWidth/2 + 0.06;  bodyHeight/2];
    robot.gc.point(:,6) = [-bodyLength/2 - 0.05;-bodyWidth/2 - 0.06; -bodyHeight/2];
    robot.gc.point(:,7) = [-bodyLength/2 - 0.05;-bodyWidth/2 - 0.06;  bodyHeight/2];
    robot.gc.point(:,8) = [-bodyLength/2 - 0.05; bodyWidth/2 + 0.06; -bodyHeight/2];
    robot.gc.point(:,9) = [-bodyLength/2 - 0.05; bodyWidth/2 + 0.06;  bodyHeight/2];

    %knee contacts
    robot.gc.point(:,10)  = [0; 0; -hipLinkLength];
    robot.gc.point(:,11)  = [0; 0; -hipLinkLength];
    %foot contacts
    % robot.gc.point(:,12) = [0;0;0];
    % robot.gc.point(:,13) = sphere_botleft;
    % robot.gc.point(:,14) = sphere_botright;
    % robot.gc.point(:,15) = sphere_topleft;
    % robot.gc.point(:,16) = sphere_topright;

    % robot.gc.point(:,17) = [0;0;0];
    % robot.gc.point(:,18) = sphere_botleft;
    % robot.gc.point(:,19) = sphere_botright;
    % robot.gc.point(:,20) = sphere_topleft;
    % robot.gc.point(:,21) = sphere_topright;
    % robot.gc.body = [ones(1,9), 3, 6, ones(1,5)*5, ones(1,5)*9]; % body + knee + foot

    robot.gc.point(:,12) = [0;0;0];
    robot.gc.point(:,13) = sphere_bot;
    robot.gc.point(:,14) = sphere_top;

    robot.gc.point(:,15) = [0;0;0];
    robot.gc.point(:,16) = sphere_bot;
    robot.gc.point(:,17) = sphere_top;

    robot.gc.body = [ones(1,9), 3, 6, ones(1,3)*5, ones(1,3)*9]; % body + knee + foot

    
    
    robot.gravity = [0; 0; -9.83];

    robot = floatbase(robot);
    %%%%%%%%%%%%%%%%%%%%%%%%%
    function matrix = flipInertialY(V)
        matrix = [V(1,1) -V(1,2) V(1,3); -V(2,1) V(2,2) -V(2,3); V(3,1) -V(3,2) V(3,3)];
    end
end
function [q] = inverse_kuka(H,myrobot)
    %input params: H := H transformation matrix to position, 
    %        myrobot := robot instance to be used by toolbox
    %output:       q := vector of joint angle variables in rad

    q = zeros(1,6);
    %extract position of end effector in frame 0
    od = H(1:3,4);
    %calculate position of spherical wrist in frame 0
    oc = od - H(1:3,1:3)*[-296.23,0,myrobot.d(6)]';
    %ex,y,z position of wrist in frame 0
    xc = oc(1);
    yc = oc(2);
    zc = oc(3);

    %calculation of constants from prelab equations
    w = sqrt(xc^2 + yc^2) - myrobot.a(1); % O1 to Oc in x0
    psi = atan2(zc-myrobot.d(1),w);
    v = sqrt((sqrt(xc^2+yc^2)-myrobot.a(1))^2 + (zc -myrobot.d(1))^2); % O1 to Oc 
    cos_beta = (myrobot.a(2)^2+v^2-myrobot.a(3)^2-myrobot.d(4)^2)/(2*myrobot.a(2)*v);
    beta = atan2(sqrt(1-cos_beta^2),cos_beta);
   
    phi = atan2(myrobot.a(3),myrobot.d(4));
    cos_alpha = (myrobot.a(2)^2 + myrobot.a(3)^2 + myrobot.d(4)^2-v^2)/(2*myrobot.a(2)*sqrt(myrobot.a(3)^2+myrobot.d(4)^2));
    alpha = atan2(sqrt(1-cos_alpha^2),cos_alpha);

    %calc of joints 1 and 3
    q(1) = atan2(yc,xc);
    q(3) = alpha - pi/2 - phi;

    %calc of joints 2
    q(2) = beta + psi;

    %calc of H for frame 3
    H0_3 = forward_kuka(myrobot,q(1:3));
    %extract rotational transform of wrist
    R0_3 = H0_3(1:3,1:3);
    %extract rotational transform of end effector
    R0_6 = H(1:3,1:3);
    %calculation of rotation of end effector relative to wrist center
    R3_6 = (R0_3')*R0_6;

    %standard calculation of spherical joints as euler angles
    q(4) = atan2(R3_6(2,3), R3_6(1,3));
    q(5) = atan2(sqrt(1 - R3_6(3,3)^2),R3_6(3,3));
    q(6) = atan2(R3_6(3,2),-R3_6(3,1));


end
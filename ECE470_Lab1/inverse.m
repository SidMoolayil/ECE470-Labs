function [q] = inverse(H,myrobot)
    %input params: H := H transformation matrix to position, 
    %        myrobot := robot instance to be used by toolbox
    %output:       q := vector of joint angle variables in rad

    q = zeros(1,6);
    %extract position of end effector in frame 0
    od = H(1:3,4);
    %calculate position of spherical wrist in frame 0
    oc = od - H(1:3,1:3)*[0,0,myrobot.d(6)]';
    %ex,y,z position of wrist in frame 0
    xc = oc(1);
    yc = oc(2);
    zc = oc(3);

    %calculation of constants from prelab equations
    gamma = asin(-myrobot.d(2)/sqrt(xc^2+yc^2));
    s = zc - myrobot.d(1);
    r = sqrt(xc^2+yc^2)*cos(gamma);
    beta = acos((myrobot.a(2)^2 + r^2 + s^2 - myrobot.d(4)^2)/(2*myrobot.a(2)*sqrt(r^2 + s^2)));
    D = (r^2 + s^2 - myrobot.a(2)^2 - myrobot.d(4)^2)/(2*myrobot.a(2)*myrobot.d(4));

    %calc of joints 1 and 3
    q(1) = atan2(yc,xc)-gamma;
    q(3) = atan2(D,sqrt(1-D^2));

    %calculation of constants from prelab equations
    psi2 = atan2(zc-myrobot.d(1),sqrt(xc^2+yc^2-myrobot.d(2)^2));
    psi1 = atan2(-myrobot.d(4)*cos(q(3)),myrobot.a(2)+myrobot.d(4)*sin(q(3)));
    
    %calc of joints 2
    q(2) = psi2 - psi1;

    %calc of H for frame 3
    H0_3 = forward(q(1:3),myrobot);
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
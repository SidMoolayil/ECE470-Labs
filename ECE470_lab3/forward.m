function Hs = forward(q, myrobot)
    % Initialize output with 4x4 Identity Matrix
    H = eye(4);
    Hs = ones(4,4,6);
    %Compute i-1 to i homogeneous transformation
    for i = 1:length(q)
        current_A = [cos(q(i)), -sin(q(i))*cos(myrobot.alpha(i)),sin(q(i))*sin(myrobot.alpha(i)), myrobot.a(i)*cos(q(i));
        sin(q(i)),cos(q(i))*cos(myrobot.alpha(i)), -cos(q(i))*sin(myrobot.alpha(i)),  myrobot.a(i)*sin(q(i));
        0, sin(myrobot.alpha(i)), cos(myrobot.alpha(i)),myrobot.d(i);
        0,0,0,1];
        H = H * current_A;
        Hs(:,:,i) = H; % Store the current H matrix
    end
end

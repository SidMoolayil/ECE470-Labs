% 4.3 Foward Kinematics
function [H] = forward_kuka(myrobot,q)
    %input params: joint := joint angles
    %            myrobot := robot instance to be used by toolbox
    %output:           H := H transformation matrix with given joint angles
    
    H = eye(4);
    % calculation of H based on induvidual link H's calc using DH param
    for i = 1:length(q)
        current_A = [cos(q(i)), -sin(q(i))*cos(myrobot.alpha(i)),sin(q(i))*sin(myrobot.alpha(i)), myrobot.a(i)*cos(q(i));
        sin(q(i)),cos(q(i))*cos(myrobot.alpha(i)), -cos(q(i))*sin(myrobot.alpha(i)),  myrobot.a(i)*sin(q(i));
        0, sin(myrobot.alpha(i)), cos(myrobot.alpha(i)),myrobot.d(i);
        0,0,0,1];
        H = H * current_A;
    end

end

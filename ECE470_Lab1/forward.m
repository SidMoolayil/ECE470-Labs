% 4.3 Foward Kinematics
function [H] = forward(joint,myrobot)
    %input params: joint := joint angles
    %            myrobot := robot instance to be used by toolbox
    %output:           H := H transformation matrix with given joint angles
    
    H = eye(4);
    % calculation of H based on induvidual link H's calc using DH param
    for i = 1:length(joint)
        current_A = [cos(joint(i)), -sin(joint(i))*cos(myrobot.alpha(i)),sin(joint(i))*sin(myrobot.alpha(i)), myrobot.a(i)*cos(joint(i));
        sin(joint(i)),cos(joint(i))*cos(myrobot.alpha(i)), -cos(joint(i))*sin(myrobot.alpha(i)),  myrobot.a(i)*sin(joint(i));
        0, sin(myrobot.alpha(i)), cos(myrobot.alpha(i)),myrobot.d(i);
        0,0,0,1];
        H = H * current_A;
    end

end

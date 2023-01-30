


%% 4.1 
function [myrobot] = mypuma560(DH)
   %input params: DH := DH table of 6 joint robot
   %output: myrobot := robot instance to be used by toolbox

   %Define links
   L1 = Link(DH(1,:)); 
   L2 = Link(DH(2,:)); 
   L3 = Link(DH(3,:)); 
   L4 = Link(DH(4,:)); 
   L5 = Link(DH(5,:)); 
   L6 = Link(DH(6,:)); 
   %return myrobot instance
   myrobot = SerialLink([L1 L2 L3 L4 L5 L6]);
end
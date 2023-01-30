
%% 4.1 
function [myrobot] = mykuka_search(delta)
   %input params: DH := DH table of 6 joint robot
   %output: myrobot := robot instance to be used by toolbox
    
   %Define DH table of KUKA from prelab with delta param
   DH = [0,400, 25, pi/2;
        0, 0 ,315, 0;
        0, 0, 35, pi/2;
        0, 365, 0, -pi/2;
        0,0,0,pi/2;
        0,161.44+delta(2),-296.23+delta(1),0];
   
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
clear all;
close all;
clc;

%%4.2 
% Define theta range
q = zeros(6,200);
theta_init = [0,0,0,pi/4,-pi/3,0];
theta_fin = [pi, pi/2, pi, 3*pi/4, pi/3, 2*pi];
for i = 1:6
   q(i,:) = linspace(theta_init(i),theta_fin(i),200);

end
q = q'; % q e R200 x 6

% Define DH table from prelab
DH = [0, 76, 0, pi/2; 
    0, -23.65, 43.23, 0;
    0, 0, 0, pi/2;
    0, 43.18, 0, -pi/2;
    0, 0, 0, pi/2; 
    0, 20, 0, 0];

%create robot instance
myrobot = mypuma560(DH);

%robot moving on path plot
figure
plot(myrobot,q);


%% 4.3

%calculate position of end effector across theta range
o = zeros(200,3);
for i = 1:200
    H1 = forward(q(i,:),myrobot);
    o(i,:) = H1(1:3,4);
end

%superimpose robot moving on path plot to check for correctness
figure
plot3(o(:,1),o(:,2),o(:,3),'r'); %path plot
hold on
plot(myrobot,q); %robot movement

%% 4.4
%check of inverse function output for given H
H = [cos(pi/4) -sin(pi/4) 0 20; sin(pi/4) cos(pi/4) 0 23; 0 0 1 15; 0 0 0 1];
Q = inverse(H,myrobot) %q should = [-0.0331,-1.0667,1.0283,3.1416,3.1032,0.8185]

%generating straight path d of end effector
d = zeros(3,100);
d_init = [10,23,15]; %start position
d_fin = [30,30,100]; %end position
for i = 1:3
   d(i,:) = linspace(d_init(i),d_fin(i),100);
end
d = d';

%defining R of H matrix along path
R = [cos(pi/4), -sin(pi/4), 0;
    sin(pi/4), cos(pi/4), 0;
    0, 0, 1];

q = zeros(100,6);
H = eye(4);
for i = 1:100
    %generating current H
    H(1:3,1:3) = R;
    H(1:3,4) = d(i,:);
    %inverse kinematics calc of joint angles for current pose 
    q(i,:) = inverse(H,myrobot);
end

%superimpose robot moving on path plot to check for correctness
figure
plot3(d(:,1),d(:,2),d(:,3),'r') %path plot
hold on
plot(myrobot,q) %robot movement
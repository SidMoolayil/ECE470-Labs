%%section 4.3

%calculation of delta vector that has minimum
delta = fminunc(@deltajoint,[0,0]);% delta =  -0.7911    0.9071

% create robot instance with delta
myrobot = mykuka_search(delta);
                         
% +---+-----------+-----------+-----------+-----------+-----------+
% | j |     theta |         d |         a |     alpha |    offset |
% +---+-----------+-----------+-----------+-----------+-----------+
% |  1|         q1|        400|         25|     1.5708|          0|
% |  2|         q2|          0|        315|          0|          0|
% |  3|         q3|          0|         35|     1.5708|          0|
% |  4|         q4|        365|          0|    -1.5708|          0|
% |  5|         q5|          0|          0|     1.5708|          0|
% |  6|         q6|    162.347|   -297.021|          0|          0|
% +---+-----------+-----------+-----------+-----------+-----------+

%validation
H = [0,0,1,483.95;
0,-1,0,-165.37;
1,0,0,27.22;
0,0,0,1];
q = inverse_kuka(H,myrobot);
setAngles(q,0.04);

%%section 4.4
p_workspace =[600;100;10];
p_baseframe = FrameTransformation(p_workspace);
R = [0 0 1; 0 -1 0; 1 0 0];
H = [R p_baseframe; zeros(1,3) 1];
q = inverse_kuka(H,myrobot); % 0.2245    0.8375   -0.3132    0.2579    1.0609   -0.1280
setAngles(q,0.04);

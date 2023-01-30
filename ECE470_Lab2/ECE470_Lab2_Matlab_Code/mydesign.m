%% 4.5-3
function mydesign()
delta  =[-0.7911    0.9071]; % fminunc(@deltajoint,[0,0]);% delta =  -0.7911    0.9071

% create robot instance with delta
myrobot = mykuka_search(delta);

%pencil upright orientation
R = [0 0 1; 0 -1 0; 1 0 0];

%getting path from excel file
data = xlsread('jug.xlsx');
xdata = 550 + 10*data(:,1);
ydata = 10*data(:,2);
zdata = ones(length(data),1)*-3; %height -3 used due to insufficient contact of pencil

X_baseframe  = zeros(3,100);
figure
%visualization of pattern to confirm robot drawing
plot(xdata,ydata);

for i = 1:length(xdata)
    % transformation of path coordinates to baseframe
    X_baseframe(:,i) = FrameTransformation([xdata(i),ydata(i),zdata(i)]');
    % configuration of H with upright pencil along path
    H = [R X_baseframe(:,i); zeros(1,3) 1];
    % calc of joint angles for particular position
    q = inverse_kuka(H,myrobot);
    % setting KUKA joints to calcultated angles
    setAngles(q,0.04);
end

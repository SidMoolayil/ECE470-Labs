function mycirlcle()
delta  =[-0.7911    0.9071]; % fminunc(@deltajoint,[0,0]);% delta =  -0.7911    0.9071

% create robot instance with delta
myrobot = mykuka_search(delta);
centre =[620 0 -3]; %Define center
r = 50; %Define radius

% theta range, full circle
theta = linspace(0,2*pi,100);

% creating circle line drawing path
X_workspace = zeros(3,100);
X_workspace(1,:) = r*cos(theta)+centre(1);
X_workspace(2,:) = r*sin(theta)+centre(2);
X_workspace(3,:)=centre(3) ;

X_baseframe  = zeros(3,100);
% create robot instance with delta
R = [0 0 1; 0 -1 0; 1 0 0];

for i = 1:100
    % transformation of path coordinates to baseframe
    X_baseframe(:,i) = FrameTransformation(X_workspace(:,i));
    % configuration of H with upright pencil along path
    H = [R X_baseframe(:,i); zeros(1,3) 1];
    % calc of joint angles for particular position
    q = inverse_kuka(H,myrobot);
    % setting KUKA joints to calcultated angles
    setAngles(q,0.04);
end

end

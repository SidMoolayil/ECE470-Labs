function mysegment()
delta  =[-0.7911    0.9071]; % fminunc(@deltajoint,[0,0]);% delta =  -0.7911    0.9071

% create robot instance with delta
myrobot = mykuka_search(delta);

% creating straight line drawing path
X_workspace = zeros(3,100);
X_workspace(1,:) = 620;
X_workspace(2,:) = linspace(-100,100,100);
X_workspace(3,:)= -1;

X_baseframe  = zeros(3,100);
%pencil upright orientation
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

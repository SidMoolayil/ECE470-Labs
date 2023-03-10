function [obs] = setupobstacle412()
% Units are mm

% Obstacle 1: ground plane
obs{1}.rho0 = 150;
obs{1}.type = 'plane';

% Obstacle 2: Cylinder 1
obs{2}.R = 100;
obs{2}.c = [620;0];
obs{2}.rho0 = 150;
obs{2}.h = 572;
obs{2}.type = 'cyl';

% Obstacle 3: Cylinder 2
obs{3}.R = 100;
obs{3}.c = [620;-440];
obs{3}.rho0 = 150;
obs{3}.h = 572;
obs{3}.type = 'cyl';

end
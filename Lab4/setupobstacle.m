% Cylinder 1 Obstacle
obs{1}.R = 1/8*100;
obs{1}.c = [0.2;0.8]*100;
obs{1}.rho0 = 1/4*100;
obs{1}.h = 2*100;
obs{1}.type = 'cyl';
% Cylinder 2 Obstacle
obs{2}.R = 1/8*100;
obs{2}.c = [-0.2;-0.8]*100;
obs{2}.rho0 = 1/4*100;
obs{2}.h = 2*100;
obs{2}.type = 'cyl';
% Sphere 1 Obstacle
obs{3}.R = 1/16*100;
obs{3}.c = [0.2;0.2;1.1;]*100;
obs{3}.rho0 = 1/4*100;
obs{3}.type = 'sph';
% Sphere 2 Obstacle
obs{4}.R = 1/16*100;
obs{4}.c = [-0.2;-0.2;1.1;]*100;
obs{4}.rho0 = 1/4*100;
obs{4}.type = 'sph';
% Sphere 3 Obstacle
obs{5}.R = 1/16*100;
obs{5}.c = [0.1;0.1;0.5;]*100;
obs{5}.rho0 = 1/4*100;
obs{5}.type = 'sph';
% Sphere 4 Obstacle
obs{6}.R = 1/16*100;
obs{6}.c = [-0.1;-0.1;0.5;]*100;
obs{6}.rho0 = 1/4*100;
obs{6}.type = 'sph';
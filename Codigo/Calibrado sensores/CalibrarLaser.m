%% Calibrar Laser
clearvars;
% ------------------------------------------------------------------------
%% Inicializacion de variables
% Posicion balizas
load('balizas_calibracion.mat')

% Posicion robot 
robot.name = 'Marvin';
robot.laserName = 'LMS100';
robot.pos = [3, -2.5, 0];
robot.ang = pi/2;

% Posicion del laser respecto al robot
apoloPlaceMRobot(robot.name, robot.pos, robot.ang);
apoloUpdate();

% Numero pruebas
N = 100;

estLM = LM;

[balizasNum, medidasNum] = size(LM);
Z_real = zeros(balizasNum, medidasNum);
Z_dist = zeros(balizasNum,N);
Z_angl = zeros(balizasNum,N);

for i = 1:length(LM(:,1))
    id = 1;
    % Calculo de matriz Zk_
    incX = LM(id,1)-robot.pos(1);
    incY = LM(id,2)-robot.pos(2);
    incAng = atan2(incY,incX) - robot.ang;

    Z_real(id,1) = sqrt(incX^2+incY^2);
    Z_real(id,2) = incAng;
end

for j = 1:N
    baliza = apoloGetLaserLandMarks(robot.laserName);
    for i = 1:length(baliza.id)
        id = baliza.id(i);      
        
        Z_dist(id,j) = baliza.distance(i);
        Z_angl(id,j) = baliza.angle(i);
    end
end

std(Z_dist')
std(Z_angl')

disp(['Z_dist: std: ',  num2str(mean(std(Z_dist')))])
disp(['Z_angl: std: ',  num2str(mean(std(Z_angl')))])

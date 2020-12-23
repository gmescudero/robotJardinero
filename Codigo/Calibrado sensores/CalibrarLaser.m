%% Calibrar Laser
clear;
% ------------------------------------------------------------------------
%% Inicializacion de variables
% Posicion balizas
LM(1,:) = [-3.9, 0.0, 0.2];
LM(2,:) = [-3.9, 3.9, 0.2];
LM(3,:) = [0.0, 3.9, 0.2];
LM(4,:) = [3.9, 3.9, 0.2];
LM(5,:) = [3.9, 0.0, 0.2];
% Posicion robot 
robot.pos = [0, 0, 0];
robot.ang = [pi/2];
% Posicion del laser respecto al robot
laser.pos(1) = 0.1;
laser.pos(2) = 0;
laser.ang = 0;
apoloPlaceMRobot('Marvin', robot.pos, robot.ang);

% Pasamos la posición del laser a cordenadas refenciales
laserPosXRef = laser.pos(1)*cos(robot.ang) - laser.pos(2)*sin(robot.ang);
laserPosYRef = laser.pos(1)*sin(robot.ang) + laser.pos(2)*cos(robot.ang);
laserAngRef = robot.ang + laser.ang;

% Numero pruebas
N = 100;
% Numero de balizas a probar
K = 5;

% Inicializacion de matrices
errorX = [];
errorY = [];
errorAng = [];
x_est_laser = [];
y_est_laser = [];
ang_est_laser = [];

% Inicio de toma de medidas
for i = 1:N
    % Se realiza una busqueda de balizas por el laser
    apoloUpdate;
    baliza = apoloGetLaserLandMarks('LMS100');
    for j = 1:K
        % Extaccion de las medidas realizadas por el laser
        distancia_laser = baliza.distance(j);
        angulo_laser = baliza.angle(j);
        % Estimacion de la posicion a partir de los datos obtenidos por el
        % laser
        x_est_laser(end+1) = - distancia_laser*sin(angulo_laser) + laserPosXRef;
        y_est_laser(end+1) = distancia_laser*cos(angulo_laser) + laserPosYRef;
        ang_est_laser(end+1) = angulo_laser + laserAngRef - pi/2;
        % Calculo del error
        errorX(end+1) = LM(j,1) - x_est_laser(end);
        errorY(end+1) = LM(j,2) - y_est_laser(end);
        errorAng(end+1) = atan2(LM(j,1),LM(j,2)) + ang_est_laser(end);
    end
end

% Extraemos la media del error
e_x_med = mean(errorX);
e_y_med = mean(errorY);
e_ang_med = mean(errorAng);

% Extraemos la varianza del error
e_x_var = var(errorX);
e_y_var = var(errorY);
e_ang_var = var(errorAng);

% Mostramos por pantalla
disp('------------ Calibracion sensor ------------')
disp(['Media del error: [' num2str(e_x_med) ', ' num2str(e_y_med) ', ' num2str(e_ang_med) ']'])
disp(['Vaianza del error: [' num2str(e_x_var) ', ' num2str(e_y_var) ', ' num2str(e_ang_var) ']'])
disp('--------------------------------------------')

%% Ploteo

% Ploteo de los errores
figure(1)
subplot(3,1,1)
plot(errorX)
title('Error en X')
subplot(3,1,2)
plot(errorY)
title('Error en Y')
subplot(3,1,3)
plot(errorAng)
title('Error en angulo')

% Ploteo de las balizas y las posiciones estimadas de las mismas
figure(2)
% Dibujamos balizas
plot(LM(:,1),LM(:,2),'o')
hold on
% Dibujamos posicion del robot
plot(robot.pos(1),robot.pos(2),'kx');
for i = 1:length(x_est_laser)
    plot(x_est_laser(i), y_est_laser(i), 'r.')
end
hold off

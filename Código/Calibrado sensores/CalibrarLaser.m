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
% Posicion del laser respecto al robot
laser.pos(1) = 0.1;
laser.pos(2) = 0.4;
laser.ang = 0;

% Numero pruebas
N = 100;
% Numero de balizas a probar
K = 5;

% Inicializacion de matrices
errorX = [];
errorY = [];
x_est_laser = [];
y_est_laser = [];

% Inicio de toma de medidas
for i = 1:N
    % Se realiza una busqueda de balizas por el laser
    baliza = apoloGetLaserLandMarks('LMS100');
    for j = 1:K
        % Extaccion de las medidas realizadas por el laser
        distancia_laser = baliza.distance(j);
        angulo_laser = baliza.angle(j);
        % Estimacion de la posicion a partir de los datos obtenidos por el
        % laser
        x_est_laser(end+1) = - distancia_laser*sin(angulo_laser);
        y_est_laser(end+1) = distancia_laser*cos(angulo_laser);
        % Calculo del error
        errorX(end+1) = LM(j,1) - x_est_laser(end);
        errorY(end+1) = LM(j,2) - y_est_laser(end);
    end
end

% Extraemos la media del error
e_x_med = mean(errorX);
e_y_med = mean(errorY);

% Extraemos la varianza del error
e_x_var = var(errorX);
e_y_var = var(errorY);

% Mostramos por pantalla
disp('------------ Calibracion sensor ------------')
disp(['Media del error: [' num2str(e_x_med) ', ' num2str(e_y_med) ']'])
disp(['Desviacion del error: [' num2str(e_x_var) ', ' num2str(e_y_var) ']'])
disp('--------------------------------------------')

%% Ploteo

% Ploteo de los errores
figure(1)
subplot(2,1,1)
plot(errorX)
title('Error en X')
subplot(2,1,2)
plot(errorY)
title('Error en Y')

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

%% Robot Jardinero
clearvars; clc;

%% Config
h = 0.25;    % Actualizacion de sensores
tmax = 200;

% max vels 
vMax = 2;
wMax = 2;

% Posicion robot
robot.name = 'Marvin';
robot.pos  = [1, 1, 0];
robot.ang  = [0];

% Waypoints 
wp = [... wp(xPos,yPos)
%     [ 2  2];
%     [-2  2];
%     [-2 -2];
%     [ 2 -2];
    [    1.0000    1.0000];
    [    5.0055    2.9653];
    [   13.5643   13.2041];
    [   24.3775   14.1838];
    [   31.4076   20.3689];
    [   41.4418   29.5726];
    [   46.8712   26.3391];
    [   55.1523   26.5495];
    [   64.2395   29.6187];
    [   74.0155   30.8810];
    [   86.6054   31.9431];
    [  101.4533   33.1906];
    [  112.2420   37.1852];
    [  120.4202   35.0337];
    [  122.5935   27.6108];
    [  120.0000   30.0000];
];
wp = wp.*[(26.5/125) (13.5/65)]; % adjust limits

% Controller parameters
% controller.Kp       = 0.15;
% controller.Ki       = 0;
% controller.Kd       = 0.10;
controller.Kp       = 15;
controller.Ki       = 0;
controller.Kd       = 0.10;
controller.sampleT  = h;

%% Initialization
% Timing params
tini = 0;
tAcum = tini:h:tmax;
t = tini;

% Velocities
v = 0.0;    % Velocidad lineal
w = 0.0;    % Velocidad angular

% Place robot on init position
apoloPlaceMRobot(robot.name, robot.pos, robot.ang);
apoloUpdate();

% Inicializamos la posición inicial
Xrealk = [robot.pos(1); robot.pos(2); robot.ang];
Xk = Xrealk;

% Posicion del laser respecto al robot
laser.name = 'LMS100';
laser.pos(1) = 0;
laser.pos(2) = 0;
laser.ang    = 0;

% Pasamos la posición del laser a cordenadas refenciales
laserPosXRef = laser.pos(1)*cos(robot.ang) - laser.pos(2)*sin(robot.ang);
laserPosYRef = laser.pos(1)*sin(robot.ang) + laser.pos(2)*cos(robot.ang);
laserAngRef = robot.ang + laser.ang;

% Inicializacion matriz P
Pxini       = 1e-1;
Pyini       = 1e-1;
Pthetaini   = 1e-1;
Pk = diag([Pxini,Pyini,Pthetaini]);

% Posicion balizas
% LM(xPos,yPos)
load('balizas_jardin.mat');

% Controlador
se = 0;
e_ = 0;

%% Algorithm
wpind = 1;
Ktotal = zeros(3);

ret = 1;
% TODO planification
for k = 1:length(tAcum)    
    % Retrieve the robot location from Kalman filter
    [Xk,Pk] = getLocation(robot.name,laser.name,LM,Xk,Pk,h,v,w);

    % Trajectory controller
    [vControll,wControll,wpReached] = controllerPID(controller,Xk,wp(wpind,:));
    
    % Set next waypoint
    if wpReached
        wpind = wpind + 1; 
        % TODO planification
    end
    
    % Reactive control
    [vReact, wReact] = reactiveControl();
    
    % Compute velocity
    v = vControll*vReact*vMax;
    w = wControll*wReact*wMax;
    
    % Movement
    ret = apoloMoveMRobot(robot.name, [v w], h);
    if 0 == ret
        tAcum = tini:h:t-h;
        break
    end
    
    % New iteration
    t = t+h;
    apoloUpdate();
%     pause(h/2);

    %% Data adquisition
    XrealAUX = apoloGetLocationMRobot(robot.name);
    
    % Obtenemos la posicion real del robot
    Xrealk(1) = XrealAUX(1);
    Xrealk(2) = XrealAUX(2);
    Xrealk(3) = XrealAUX(4);
    Xreal(:,k) = Xrealk;        % Para mantener una historia del recorrido
        
    %Sólo para almacenarlo
    Xestimado(:,k) = Xk;
end

%% Ploting and such
% Representacion grafica
figure(1);
subplot(3,2,[1 3 5])
axis([-4 4 -4 4])

hold on
plot(Xreal(1,:), Xreal(2,:), 'r');
% plot(Xestimado(1,:), Xestimado(2,:), '--b');
for i=1:t/h
    plot(Xestimado(1,i), Xestimado(2,i), '.b');
end
hold off
title('Trayectoria')
xlabel('x(m)')
ylabel('y(m)')
legend('Movimiento Real','Estimación')

subplot(3,2,2)
plot(tAcum,Xreal(1,:),'r');
hold on
plot(tAcum,Xestimado(1,:),'.b');
hold off
xlabel('t(s)')
ylabel('x(m)')

subplot(3,2,4)
plot(tAcum,Xreal(2,:),'r');
hold on
plot(tAcum,Xestimado(2,:),'.b');
hold off
xlabel('t(s)')
ylabel('y(m)')

subplot(3,2,6)
plot(tAcum,Xreal(3,:),'r');
hold on
plot(tAcum,Xestimado(3,:),'.b');
hold off
xlabel('t(s)')
ylabel('\theta(m)')
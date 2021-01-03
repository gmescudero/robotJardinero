%% Robot Jardinero
clearvars; clc; close all;

%% Config
h = 0.25; % Refresh rate

% max vels 
vMax = 1;
wMax = 1;
vMin = 0.05;
wMin = 0.05;

% Planning 
goal = [26, 6];
xSize = 26.5;
iterationLim = 80; % Max loops for waypoint
maxDistance  = 5;  % Max distance between waypoints

% Posicion robot
robot.name = 'Marvin';
robot.pos  = [0.5,0.5, 0];
robot.ang  = 0;

% Controller parameters
controller.Kp       = 0.15;
controller.Ki       = 0;
controller.Kd       = 0.10;
controller.sampleT  = h;

%% Initialization
% Timing params
tini = 0;
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

% Planning
wpind = 1;
dist = 0;

k   = 0;
wpK = 0;
%% Algorithm

% Planification
load jardinBinMap2.mat 
cellsPerMeter = length(BW(1,:))/xSize;
[ret,wp] = mappingAndPlan(2,BW,Xk,goal,cellsPerMeter);

while (wpind < length(wp)) && (0 ~= ret)
    k=k+1;
    
    % Retrieve the robot location from Kalman filter
    [Xk,Pk] = getLocation(robot.name,laser.name,LM,Xk,Pk,h,v,w);

    % Trajectory controller
    [vControl,wControl,wpReached] = controllerPID(controller,Xk,wp(wpind,:));
    
    % Set next waypoint
    if wpReached || dist > maxDistance || k-wpK > iterationLim
        wpind = wpind + 1; 
        dist = 0;
        vControl = 0;
        wControl = 0;
        wpK = k;
    end
    
    % Reactive control
    [vReact, wReact,tooClose] = reactiveControl();
    
    % Compute velocity
    if tooClose
        v = vReact*vMax;
        w = wReact*wMax;
    else
        v = vControl*vMax;
        w = wControl*wMax;
    end
    dist = dist + v*h;
    
    % Movement
    ret = apoloMoveMRobot(robot.name, [v w], h);
    
    % New iteration
    t = t+h;
    apoloUpdate();
    pause(h/2);

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

%% Ploting
tAcum = 0:h:(k-1)*h;

% Representacion grafica
figure(1);
subplot(3,2,[1 3 5])
axis([-0.5 27 -0.5 14])

hold on
plot(Xreal(1,:), Xreal(2,:), 'r');
% plot(Xestimado(1,:), Xestimado(2,:), '--b');
for i=1:t/h
    plot(Xestimado(1,i), Xestimado(2,i), '.b');
end
for i = 1:length(wp(:,1))
    plot(wp(i,1), wp(i,2), 'o');
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

figure(2)

imshow (~BW)
hold on
plot(Xreal(1,:)*cellsPerMeter, Xreal(2,:)*cellsPerMeter, 'r');
for i = 1:length(wp(:,1))
    x = wp(i,1)*cellsPerMeter;
    y = wp(i,2)*cellsPerMeter;
    plot(x,y, 'o');
end
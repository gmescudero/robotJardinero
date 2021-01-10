%% Robot Jardinero
clearvars; clc; close all;

%% Config
h = 0.1; % Refresh rate
tmax = 500;

% max vels 
vMax = 0.5;
wMax = 0.5;

% Planning 
% goal = [12, 6];
planningAlgorithm = 1; % RRT = 1, PRM = 2
goal = [26, 6];
xSize = 26.5;
iterationLim = 20; % Max loops for waypoint
maxDistance  = 5;  % Max distance between waypoints

% Posicion robot
robot.name = 'Marvin';
robot.pos  = [0.5,0.5, 0];
robot.ang  = 0;

% Controller parameters
controller.Kp        = 0.25;
controller.Ki        = 0;
controller.Kd        = 0.6;
controller.sampleT   = h;
controller.reachedTh = 0.2;

fisFile = 'mobileController.fis';
controlfuzzy = readfis(fisFile);
tgtRange = 2;

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

% Inicializacion matriz P
Pxini       = 1e-1;
Pyini       = 1e-1;
Pthetaini   = 1e-1;
Pk = diag([Pxini,Pyini,Pthetaini]);

% Posicion balizas
% LM(xPos,yPos)
load('balizas_jardin.mat');

% Mapa binario
% BW(x,y)
% load jardinBinMapCorrectAndDilated.mat
load jardinBinMap3.mat

% Controlador
se = 0;
e_ = 0;

% Planning
dist = 0;

k   = 0;
wpK = 0;
%% Algorithm

% Planification
cellsPerMeter = fix(length(BW(1,:))/xSize);
disp('Planning...');
tic
% RRT
[ret,wp] = mappingAndPlan(1,BW,Xk,goal,cellsPerMeter);
if 0 == ret
    % PRM
    [ret,wp] = mappingAndPlan(2,BW,Xk,goal,cellsPerMeter);
end
toc
disp('Planning finished');

reaktK = 0;
wpind = 1;

% wp = [goal];

while (wpind <= length(wp)) && (0 ~= ret) && (t < tmax)
    k=k+1;
    
    % Retrieve the robot location from Kalman filter
    [Xk,Pk] = getLocation(robot.name,laser.name,LM,Xk,Pk,h,v,w);

    tgtDist = sqrt((wp(wpind,1) - Xk(1))^2 + (wp(wpind,2)- Xk(2))^2);
    tgtAngl = atan2((wp(wpind,2)- Xk(2)),(wp(wpind,1) - Xk(1)));
    
    % Check if replan is needed
    if tgtDist > 2*tgtRange
        [ret,wp] = mappingAndPlan(2,BW,Xk,goal,cellsPerMeter);
        if 0 == ret
            break
        else
            wpind = 1;
        end
    else
        % Choose next waypoint
        while tgtDist < tgtRange && wpind < length(wp(:,1))
            wpind = wpind+1;
            % Calculate target dist and angle
            tgtDist = sqrt((wp(wpind,1) - Xk(1))^2 + (wp(wpind,2)- Xk(2))^2);
            tgtAngl = atan2((wp(wpind,2)- Xk(2)),(wp(wpind,1) - Xk(1)));
        end 
        
        % Get ultrasonic measures
        uSonic = [...
            [apoloGetUltrasonicSensor('uci'),pi/2];
            [apoloGetUltrasonicSensor('ucd'),-pi/2];
            [apoloGetUltrasonicSensor('ul'),pi];
            [apoloGetUltrasonicSensor('ur'),-pi];
            ];
        
        % Choose the nearest one
        ind = 1;
        obsDist = uSonic(1,1);
        obsAngl = uSonic(1,2);
        for i = 2:length(uSonic(:,1))
            if uSonic(i,1) < obsDist
                obsDist = uSonic(i,1);
                obsAngl = uSonic(i,2);
            end
        end
        
        % Limit distance to 1
        if obsDist > 1
            obsDist = 1;
            obsAngl = 0;
        end
        
        % Fuzzy controller
        out = evalfis(controlfuzzy,[obsDist,obsAngl,tgtDist,tgtAngl]);
        w = out(1)*vMax;
        v = out(2)*wMax;
        
        % Movement
        ret = apoloMoveMRobot(robot.name, [v w], h);
    end
    % New iteration
    t = t+h;
    apoloUpdate();
    pause(h/4);

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

if 0 == ret
    disp('Trajectory collision!');
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
x = wp(wpind,1)*cellsPerMeter;
y = wp(wpind,2)*cellsPerMeter;
plot(x,y, '*');

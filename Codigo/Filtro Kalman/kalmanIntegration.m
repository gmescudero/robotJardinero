%% Filtro Kalman
clearvars; clc;

% Definimos una trayectoria circular
h = 0.25;    % Actualizacion de sensores
v = 0.0;    % Velocidad lineal
w = 0.0;    % Velocidad angular

% Posicion robot
robot.name = 'Marvin';
robot.pos  = [2, 0, 0];
robot.ang  = [pi/2];
apoloPlaceMRobot(robot.name, robot.pos, robot.ang);

% Inicializamos la posición inicial
Xrealk = [robot.pos(1); robot.pos(2); robot.ang];
Xk = Xrealk;

% Posicion del laser respecto al robot
laser.name = 'LMS100';
laser.pos(1) = 0;
laser.pos(2) = 0;
laser.ang = 0;

% Pasamos la posición del laser a cordenadas refenciales
laserPosXRef = laser.pos(1)*cos(robot.ang) - laser.pos(2)*sin(robot.ang);
laserPosYRef = laser.pos(1)*sin(robot.ang) + laser.pos(2)*cos(robot.ang);
laserAngRef = robot.ang + laser.ang;

% Inicializacion matriz P
Pxini       = 1e-3;
Pyini       = 1e-3;
Pthetaini   = 1e-3;
Pk = diag([Pxini,Pyini,Pthetaini]);

% Posicion balizas
LM = [... LM(xPos,yPos)
    [-3.9,  0.0];
    [-3.9,  3.9];
    [ 0.0,  3.9];
    [ 3.9,  3.9];
    [ 3.9,  0.0];
    [ 3.9, -2.9];
    [ 0.0, -2.9];
    [-3.9, -2.9];
];
% Waypoint final
wp = [... wp(xPos,yPos)
    [ 2  2];
    [-2  2];
    [-2 -2];
    [ 2 -2];
];

% Controlador
Kp = 0.15;
Ki = 0;
Kd = 0.10;
se = 0;
e_ = 0;

% Algoritmo
tini = 0;
tmax = 200;
tAcum = tini:h:tmax;
t = tini;
wpind = 1;
Ktotal = zeros(3);

ret = 1;
for k = 1:length(tAcum)
    % Comprueba si esta en el wp y si es asi pasa al siguiente wp
    if sqrt((wp(wpind,2)-Xk(2))^2+(wp(wpind,1)-Xk(1))^2)<0.1
        wpind = wpind+1;
        if wpind > length(wp)
            wpind = 1;
        end
    end
    
    % Controlador PID
    angwp = atan2(wp(wpind,2)-Xk(2),wp(wpind,1)-Xk(1));
    
    e =(sin(angwp-Xk(3)) + (1-cos(angwp-Xk(3))));
    se = se + e;
    w = Kp*e + Ki*h*se + Kd*(e-e_)/h;
    e_ = e;
    
    if abs(angwp-Xk(3)) < pi/10
        v = 0.2;
    else
        v = 0.075;
    end
    
    % Avance real del robot
    ret = apoloMoveMRobot(robot.name, [v w], h);
    if 0 == ret
        tAcum = tini:h:t-h;
        break
    end
    XrealAUX = apoloGetLocationMRobot(robot.name);
    
    % Obtenemos la posicion real del robot
    Xrealk(1) = XrealAUX(1);
    Xrealk(2) = XrealAUX(2);
    Xrealk(3) = XrealAUX(4);
    Xreal(:,k) = Xrealk;        % Para mantener una historia del recorrido
    
    [Xk,Pk] = getLocation(robot.name,laser.name,LM,Xk,Pk,h,v,w);
    
    %Sólo para almacenarlo
    Xestimado(:,k) = Xk;
    Pacumulado(1,k) = Pk(1,1);
    Pacumulado(2,k) = Pk(2,2);
    Pacumulado(3,k) = Pk(3,3);
    
    % Actualizacion del tiempo
    t = t+h;
    apoloUpdate;
%     pause(h-tDelta)
end

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




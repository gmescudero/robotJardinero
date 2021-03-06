%% Filtro Kalman
clearvars; clc;

% Definimos una trayectoria circular
h = 0.5;    % Actualizacion de sensores
v = 0.01;    % Velocidad lineal
w = -0.01;    % Velocidad angular

% Posicion robot 
robot.pos = [-2.5, 0, 0];
robot.ang = [-pi/4];
apoloPlaceMRobot('Marvin', robot.pos, robot.ang);

% Inicializamos la posici�n inicial
Xrealk = [robot.pos(1); robot.pos(2); robot.ang];
Xk = [-2.5; 0; -pi/4];

% Posicion del laser respecto al robot
laser.pos(1) = 0;
laser.pos(2) = 0;
laser.ang = 0;

% Pasamos la posici�n del laser a cordenadas refenciales
laserPosXRef = laser.pos(1)*cos(robot.ang) - laser.pos(2)*sin(robot.ang);
laserPosYRef = laser.pos(1)*sin(robot.ang) + laser.pos(2)*cos(robot.ang);
laserAngRef = robot.ang + laser.ang;

% Varianza del ruido del proceso 
Qv = 0.0150;
Qw = 0.0190;
Qk_1 = [Qv 0; 0 Qw];

% Inicializacion matriz P
Pxini = 0.013;
Pyini = 0.010;
Pthetaini = 0.010;
Pk = [Pxini 0 0; 0 Pyini 0; 0 0 Pthetaini];

% Varianza en la medida
R1 = 0.0136;
R2 = 0.0089;
R3 = 0.0100;

% Posicion balizas
LM(1,:) = [-3.9, 0.0, 0.2];
LM(2,:) = [-3.9, 3.9, 0.2];
LM(3,:) = [0.0, 3.9, 0.2];
LM(4,:) = [3.9, 3.9, 0.2];
LM(5,:) = [3.9, 0.0, 0.2];
LM(6,:) = [3.9, -2.9, 0.2];
LM(7,:) = [0, -2.9, 0.2];
LM(8,:) = [-3.9, -2.9, 0.2];

% Waypoints
wp(1,:) = [2 2];
wp(2,:) = [-2 2];
wp(3,:) = [-2 -2];
wp(4,:) = [2 -2];

% Controlador
Kp = 0.25;
Ki = 0.0;
Kd = 0.1;
se = 0;
e_ = 0;

% Algoritmo
t = 0;
tmax = 500;
tAcum = [];
k = 1;
wpind = 1;
Ktotal = zeros(3);  
Eacumulado1 = [];
Eacumulado2 = [];
Eacumulado3 = [];
X_realk = robot.pos;
while t<tmax
    
    % Comprueba si esta en el wp y si es asi pasa al siguiente wp
    if sqrt((wp(wpind,2)-Xk(2))^2+(wp(wpind,1)-Xk(1))^2)<0.2
        wpind = wpind+1;
        if wpind > length(wp)
            wpind = 1;
        end
    end
    
%     % Controlador PID
%     angwp = atan2(wp(wpind,2)-Xk(2),wp(wpind,1)-Xk(1));
%     e =(sin(angwp-Xk(3)) + (1-cos(angwp-Xk(3))));
%     se = se + e;
%     w = Kp*e + Ki*h*se + Kd*(e-e_)/h;
%     e_ = e;  
%     if abs(angwp-Xk(3)) < pi/10
%         v = 0.2;
%     else
%         v = 0.1;
%     end
    
    % Avance real del robot
    apoloMoveMRobot('Marvin', [v w], h);
    XrealAUX = apoloGetLocationMRobot('Marvin');
    
    % Obtenemos la posicion real del robot
    Xrealk(1) = XrealAUX(1);
    Xrealk(2) = XrealAUX(2);
    Xrealk(3) = XrealAUX(4);
    Xreal(:,k) = Xrealk;        % Para mantener una historia del recorrido
 
    % Se realiza una busqueda de balizas por el laser
    Zk = [];
    baliza = apoloGetLaserLandMarks('LMS100');    
    for j = 1:length(baliza.distance)
        id = baliza.id(j);
        % Extraccion de las medidas realizadas por el laser
        distancia_laser = baliza.distance(j);
        angulo_laser = baliza.angle(j);
        Zk(3*j-2,1) = distancia_laser;
        Zk(3*j-1,1) = sin(angulo_laser);
        Zk(3*j-0,1) = cos(angulo_laser);
        % temporal para calibracion
        incX_cal = LM(id,1) - Xrealk(1);
        incY_cal = LM(id,2) - Xrealk(2);
        ang_cal = atan2(incY_cal,incX_cal) - Xrealk(3);
        Zreal1(j,1) = sqrt(incX_cal^2+incY_cal^2);
        Zreal2(j,1) = sin(ang_cal);
        Zreal3(j,1) = cos(ang_cal);
        Eacumulado1(end+1,1) = Zk(3*j-2,1) - Zreal1(j,1);
        Eacumulado2(end+1,1) = Zk(3*j-1,1) - Zreal2(j,1);
        Eacumulado3(end+1,1) = Zk(3*j-0,1) - Zreal3(j,1);
    end

    % Nuevo ciclo, k-1 = k.
    Xk_1 = Xk;
    Pk_1 = Pk;
    
    % Prediccion del estado (Modelo de odometria)
    X_k = [(Xk_1(1) + v*h*cos(Xk_1(3)+(w*h/2)));
           (Xk_1(2) + v*h*sin(Xk_1(3)+(w*h/2)));
           (Xk_1(3) + w*h)];
       
    Ak = [1 0 (-v*h*sin(Xk_1(3)+w*h/2));
          0 1 (v*h*cos(Xk_1(3)+w*h/2));
          0 0 1                             ]; % Fi
    Bk = [(cos(Xk_1(3)+w*h/2)) (-0.5*v*h*sin(Xk_1(3)+w*h/2));
          (sin(Xk_1(3)+w*h/2)) (0.5*v*h*cos(Xk_1(3)+w*h/2));
           0                     1          ]; % G
       
    P_k = Ak*Pk_1*((Ak)') + Bk*Qk_1*((Bk)');
    
    if X_k(3) <= -pi
        X_k(3) = X_k(3)+2*pi;
    elseif X_k(3) >= pi
        X_k(3) = X_k(3)-2*pi;
    end
    
    % Prediccion de la medida (Modelo de observacion)
    Zk_ = [];
    Rk_aux = [];
    Hk = [];
    Yk = [];
    for j = 1:length(baliza.distance)
        id = baliza.id(j);
        % Calculo de matriz Zk_
        incX = LM(id,1)-X_k(1);
        incY = LM(id,2)-X_k(2);
        incAng = atan2(incY,incX) - X_k(3);
        if incAng <= -pi
            incAng = incAng+2*pi;
        elseif incAng >= pi
            incAng = incAng-2*pi;
        end
        Zk_(3*j-2,1) = sqrt(incX^2+incY^2);
        Zk_(3*j-1,1) = sin(incAng);
        Zk_(3*j,1) = cos(incAng);
        % Calculo de matriz Hk
        Hk(3*j-2,:) = [incX/sqrt(incX^2+incY^2) incY/sqrt(incX^2+incY^2) 0];
        Hk(3*j-1,:) = [-incY*cos(-incAng)/(incX^2+incY^2) incX*cos(-incAng)/(incX^2+incY^2) -cos(-incAng)];
        Hk(3*j-0,:) = [incY*sin(-incAng)/(incX^2+incY^2) -incX*sin(-incAng)/(incX^2+incY^2) -sin(-incAng)];
        % Calculo de matriz Rk
        Rk_aux(3*j-2) = R1;
        Rk_aux(3*j-1) = R2;
        Rk_aux(3*j) = R3;
        
        % Calculo de matriz Yk
        % Comparacion
        Yk(3*j-2,1) = -(Zk(3*j-2,1)-Zk_(3*j-2,1));
        Yk(3*j-1,1) = +(Zk(3*j-1,1)-Zk_(3*j-1,1));
        Yk(3*j-0,1) = +(Zk(3*j-0,1)-Zk_(3*j-0,1));
    end
    Rk = diag(Rk_aux);
    % ----------------------------------------------------------------
    
    if isempty(baliza.distance)
        Xk = X_k;
        Pk = P_k;
    else
        % Comparacion
        Sk = Hk*P_k*((Hk)') + Rk;
        % Correccion
        Wk = P_k*((Hk)')/Sk;
        Xk = X_k + Wk*Yk;
        Pk = (eye(3) - Wk*Hk)*P_k;
        % Correcion de angulo
        if Xk(3) <= -pi
            Xk(3) = Xk(3)+2*pi;
        elseif Xk(3) >= pi
            Xk(3) = Xk(3)-2*pi;
        end
    end
    
    %S�lo para almacenarlo
    Xestimado(:,k) = Xk;
    
    % Temporal, solo para calibracion
    if Xrealk(3) <= -pi
        Xrealk(3) = Xrealk(3)+2*pi;
        X_realk(3) = X_realk(3)+2*pi;
    elseif Xrealk(3) >= pi
        Xrealk(3) = Xrealk(3)-2*pi;
        X_realk(3) = X_realk(3)-2*pi;
    end
    Vxestimado = (Xrealk(1) - X_realk(1))/(h*cos(Xrealk(3)/2 + X_realk(3)/2));
    Vyestimado = (Xrealk(2) - X_realk(2))/(h*sin(Xrealk(3)/2 + X_realk(3)/2));
    Vestimado(:,k) = (Vxestimado+Vyestimado)/2;
    Westimado(:,k) = (Xrealk(3) - X_realk(3))/h;
    ErrorV(:,k) = v - Vestimado(:,k);
    ErrorW(:,k) = w - Westimado(:,k);
    
%     Eacumulado1(k) = Xrealk(1) - X_k(1);
%     Eacumulado2(k) = Xrealk(2) - X_k(2);
%     Eacumulado3(k) = Xrealk(3) - X_k(3);
    
    Pacumulado(1,k) = Pk(1,1);
    Pacumulado(2,k) = Pk(2,2);
    Pacumulado(3,k) = Pk(3,3);
    
    % Actualizacion del tiempo
    t = t+h;
    tAcum(end+1) = t;
    k = k + 1;
    X_realk = Xrealk;
    apoloUpdate;
end 

% temporal para calibracion
mean(Eacumulado1)
std(Eacumulado1)
mean(Eacumulado2)
std(Eacumulado2)
mean(Eacumulado3)
std(Eacumulado3)

% mean(ErrorV(2:end))
% std(ErrorV(2:end))
% mean(ErrorW(2:end))
% std(ErrorW(2:end))

figure(5)
subplot(2,1,1)
plot(tAcum,Vestimado)
title('Velocidad lineal estimada')
subplot(2,1,2)
plot(tAcum,Westimado)
title('Velocidad angular estimada')

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
legend('Movimiento Real','Estimaci�n')

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

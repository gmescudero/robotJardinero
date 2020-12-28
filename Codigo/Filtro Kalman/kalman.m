%% Filtro Kalman
clearvars; clc;

% Definimos una trayectoria circular
h = 0.5;    % Actualizacion de sensores
v = 0.1;    % Velocidad lineal
w = 0.05;    % Velocidad angular

% Posicion robot 
robot.pos = [2, 0, 0];
robot.ang = [pi/2];
apoloPlaceMRobot('Marvin', robot.pos, robot.ang);

% Posicion del laser respecto al robot
laser.pos(1) = 0.1;
laser.pos(2) = 0;
laser.ang = 0;

% Pasamos la posición del laser a cordenadas refenciales
laserPosXRef = laser.pos(1)*cos(robot.ang) - laser.pos(2)*sin(robot.ang);
laserPosYRef = laser.pos(1)*sin(robot.ang) + laser.pos(2)*cos(robot.ang);
laserAngRef = robot.ang + laser.ang;

% Inicializamos la posición inicial y su covarianza
Xrealk = [robot.pos(1); robot.pos(2); robot.ang];
Xk = [2; 0; pi/2];

% Varianza del ruido del proceso 
Qd = 0.0180;
Qb = 0;
Qk_1 = [Qd 0; 0 Qb];

% Inicializacion matriz P
Pxini = 1e-3;
Pyini = 1e-3;
Pthetaini = 1e-3;
Pk = [Pxini 0 0; 0 Pyini 0 ; 0 0 Pthetaini];

% Varianza en la medida
R1 = 0.0140;
R2 = 0.0140;

% Posicion balizas
LM(1,:) = [-3.9, 0.0, 0.2];
LM(2,:) = [-3.9, 3.9, 0.2];
LM(3,:) = [0.0, 3.9, 0.2];
LM(4,:) = [3.9, 3.9, 0.2];
LM(5,:) = [3.9, 0.0, 0.2];

% Algoritmo
t = 0;
tmax = 35;
tAcum = [];
k = 1;
Ktotal = zeros(3);      
while t<tmax
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
        % Extaccion de las medidas realizadas por el laser
        distancia_laser = baliza.distance(j);
        angulo_laser = baliza.angle(j);
        Zk(2*j-1,1) = distancia_laser;
        Zk(2*j,1) = angulo_laser;
    end
%     Zk = [Xrealk(1); Xrealk(2); Xrealk(3)]; % No hacer caso

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
    
    if X_k(3) < -pi
        X_k(3) = X_k(3)+2*pi;
    elseif X_k(3) > pi
        X_k(3) = X_k(3)-2*pi;
    end
    
    % Prediccion de la medida (Modelo de observacion)
    Zk_ = [];
    Rk_aux = [];
    Hk = [];
    for j = 1:length(baliza.distance)
        id = baliza.id(j);
        % Calculo de matriz Zk_
        incX = LM(id,1)-X_k(1);
        incY = LM(id,2)-X_k(2);
        Zk_(2*j-1,1) = sqrt(incX^2+incY^2);
        Zk_(2*j,1) = atan2(incY,incX) - X_k(3);
        while Zk_(2*j,1)<-pi || Zk_(2*j,1)>pi
            if Zk_(2*j,1) < -pi
                Zk_(2*j,1) = Zk_(2*j,1)+2*pi;
            elseif Zk_(2*j,1) > pi
                Zk_(2*j,1) = Zk_(2*j,1)-2*pi;
            end
%             disp('angulo corregido')
        end
        % Calculo de matriz H
        Hk(2*j-1,:) = [incX/sqrt(incX^2+incY^2) incY/sqrt(incX^2+incY^2) 0];
        Hk(2*j,:) = [-incY/(incX^2+incY^2) incX/(incX^2+incY^2) -1];
        % Calculo de matriz Rk
        Rk_aux(2*j-1) = R1;
        Rk_aux(2*j) = R2;
    end
    Rk = diag(Rk_aux);
    % ----------------------------------------------------------------
    
    % Comparacion
    Yk = -Zk+Zk_;
    Sk = Hk*P_k*((Hk)') + Rk;

    % Correccion
    Wk = [];
    Wk = P_k*((Hk)')/Sk;
    Xk = X_k + Wk*Yk;
    Pk = (eye(3) - Wk*Hk)*P_k;
    
    %Sólo para almacenarlo
    if Xk(3) < -pi
        Xk(3) = Xk(3)+2*pi;
    elseif Xk(3) > pi
        Xk(3) = Xk(3)-2*pi;
    end
    Xestimado(:,k) = Xk;
%     Zk1acumulado(k) = Zk(1);
%     Zk2acumulado(k) = Zk(2);
%     Wruidoacumulado(k) = sqrt((Xrealk(1)-X_k(1))^2+(Xrealk(2)-X_k(2))^2+(Xrealk(3)-X_k(3))^2);
    Pacumulado(1,k) = Pk(1,1);
    Pacumulado(2,k) = Pk(2,2);
    Pacumulado(3,k) = Pk(3,3);
    
    % Actualizacion del tiempo
    t = t+h;
    tAcum(end+1) = t;
    k = k + 1;
    apoloUpdate;
end 

% mean(Zk1acumulado)
% std(Zk1acumulado)
% mean(Zk2acumulado)
% std(Zk2acumulado)
% std(Wruidoacumulado)

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




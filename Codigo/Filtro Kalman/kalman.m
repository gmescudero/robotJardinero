%% Filtro Kalman
clearvars; clc;

% Definimos una trayectoria circular
h = 0.5;    % Actualizacion de sensores
v = 0.1;    % Velocidad lineal
w = 0.1;    % Velocidad angular

% Posicion robot 
robot.pos = [0, 0, 0];
robot.ang = [pi/2];

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
Xk = [0; 0; pi/2];
apoloPlaceMRobot('Marvin', robot.pos, robot.ang);

% Varianza del ruido del proceso 
Qd = 0.001*v;
Qb = 0.001*w;
Qk_1 = [Qd 0; 0 Qb];

% Inicializacion matriz P
Pxini = 0.001;
Pyini = 0.001;
Pthetaini = 0.001;
Pk = [Pxini 0 0; 0 Pyini 0 ; 0 0 Pthetaini];

% Varianza en la medida
% R1 = 0.0016278;
% R2 = 0.0020709;
% R3 = 0.00049331;
% Rk = [R1 0 0; 0 R2 0; 0 0 R3];

% Posicion balizas
LM(1,:) = [-3.9, 0.0, 0.2];
LM(2,:) = [-3.9, 3.9, 0.2];
LM(3,:) = [0.0, 3.9, 0.2];
LM(4,:) = [3.9, 3.9, 0.2];
LM(5,:) = [3.9, 0.0, 0.2];

% Algoritmo
t = 0;
tmax = 30;
k = 1;
Ktotal = zeros(3);      
while t<tmax
    % Avance real del robot
    apoloMoveMRobot('Marvin', [v w], h);
    XrealAUX = apoloGetLocationMRobot('Marvin');
    
    Xrealk(1) = XrealAUX(1);
    Xrealk(2) = XrealAUX(2);
    Xrealk(3) = XrealAUX(4);
    Xreal(:,k) = Xrealk;  % Para mantener una historia del recorrido
 
    % Se realiza una busqueda de balizas por el laser
    Zk = [];
    baliza = apoloGetLaserLandMarks('LMS100');    
    for j = 1:length(baliza.distance)
        id = baliza.id(j);
        % Extaccion de las medidas realizadas por el laser
        distancia_laser = baliza.distance(j);
        angulo_laser = baliza.angle(j);
        Zk(2*j-1) = distancia_laser;
        Zk(2*j) = angulo_laser;
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
           0                     1                                 ]; % G
       
    P_k = Ak*Pk_1*((Ak)') + Bk*Qk_1*((Bk)');

    % Prediccion de la medida (Modelo de observacion)
    % ----------------- (ESTO POSIBLEMENTE ESTE MAL) -----------------
    % Depende de que modelo de lectura de balizas estemos usando
    Zk_ = [];
    Rk_aux = [];
    Hk = [];
    for j = 1:length(baliza.distance)
        id = baliza.id(j);
        incX = LM(id,1)-X_k(1);
        incY = LM(id,2)-X_k(2);
        Zk_(2*j-1) = sqrt((incX)^2 + (incY)^2);
        Zk_(2*j) = X_k(3)-atan2(incY,incX);
        Hk(2*j-1,:) = [incX/(incX^2+incY^2) incY/(incX^2+incY^2) -1];
        Hk(2*j,:) = [0 0 1];
        Rk_aux(2*j-1) = 0.001;
        Rk_aux(2*j) = 0.001;
    end
    Rk = diag(Rk_aux);
    % ----------------------------------------------------------------
    
    % Comparacion
    Yk = Zk-Zk_;
    Sk = Hk*P_k*((Hk)') + Rk;
    Wk = P_k*((Hk)')/Sk;

    % Correccion
    Xk = X_k + Wk*Yk';
    Pk = (eye(3) - Wk*Hk)*P_k;
    
    %Sólo para almacenarlo
    Xestimado(:,k) = Xk;
    Pacumulado(1,k) = Pk(1,1);
    Pacumulado(2,k) = Pk(2,2);
    Pacumulado(3,k) = Pk(3,3);
    
    % Actualizacion del tiempo
    t = t+h;
    k = k + 1;
    apoloUpdate;
end 

% Representacion grafica
figure(1);
axis([-4 4 -4 4])
hold on
plot(Xreal(1,:), Xreal(2,:), 'r');
% plot(Xestimado(1,:), Xestimado(2,:), '--b');
for i=1:t/h
   plot(Xestimado(1,i), Xestimado(2,i), '.b');
end
hold off
legend('Movimiento Real','Estimación')



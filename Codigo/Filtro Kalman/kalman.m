% Definimos una trayectoria circular
v = 0.1;  % Velocidad lineal 0.2 m/seg
w = 0;
h = 0.05;  % Actualizacion de sensores

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
Xk = [6; 3; pi];
apoloPlaceMRobot('Marvin', robot.pos, robot.ang);

% Varianza del ruido del proceso 
Qd = 0.01*v*h;
Qb = 0.02*w*h;
Qk_1 = [Qd 0; 0 Qb];

Pxini = 0.001;
Pyini = 0.001;
Pthetaini = 0.001;
Pk = [Pxini 0 0; 0 Pyini 0 ; 0 0 Pthetaini];

% Varianza en la medida
R1 = 0.001;
R2 = 0.001;
R3 = 0.001;
Rk = [R1 0 0; 0 R2 0; 0 0 R3];

% Posicion balizas
LM(1,:) = [-3.9, 0.0, 0.2];
LM(2,:) = [-3.9, 3.9, 0.2];
LM(3,:) = [0.0, 3.9, 0.2];
LM(4,:) = [3.9, 3.9, 0.2];
LM(5,:) = [3.9, 0.0, 0.2];

% Algoritmo
t = 0;
tmax = 1;
k = 0;
Ktotal = zeros(3);      
while t<tmax
    k = k + 1;
    % Avance real del robot
    apoloMoveMRobot('Marvin', [v w], h);
    XrealAUX = apoloGetLocationMRobot('Marvin');
    
    Xrealk(1) = XrealAUX(1);
    Xrealk(2) = XrealAUX(2);
    Xrealk(3) = XrealAUX(4);
    Xreal(:,k) = Xrealk;  % Para mantener una historia del recorrido

      
    % Se realiza una busqueda de balizas por el laser
    baliza = apoloGetLaserLandMarks('LMS100');
    x_sum = 0;
    y_sum = 0;
    ang_sum = 0;
    for j = 1:size(baliza.id)
        % Extaccion de las medidas realizadas por el laser
        distancia_laser = baliza.distance(j);
        angulo_laser = baliza.angle(j);
        % Estimacion de la posicion a partir de los datos obtenidos por el
        % laser
        x_est_laser = - distancia_laser*sin(angulo_laser) + laserPosXRef;
        y_est_laser = distancia_laser*cos(angulo_laser) + laserPosYRef;
        ang_est_laser = angulo_laser + laserAngRef;
        % Media estimacion
        x_sum = x_sum + x_est_laser;
        y_sum = y_sum + y_est_laser;
        ang_sum = ang_sum + ang_est_laser;
    end
    
    Zk = [x_sum/j; y_sum/j; ang_sum/j]; 

    % Nuevo ciclo, k-1 = k.
    Xk_1 = Xk;
    Pk_1 = Pk;
    
    % Prediccion del estado (Modelo de odometria)
    X_k = [(Xk_1(1) + v*cos(Xk_1(3)+w/2));
           (Xk_1(2) + v*sin(Xk_1(3)+(w/2)));
           (Xk_1(3) + w)];
       
    Ak = [1 0 (-v*sin(Xk_1(3)+w/2));
          0 1 (v*cos(Xk_1(3)+w/2));
          0 0 1                             ]; % Fi
      
    Bk = [(cos(Xk_1(3)+w/2)) (-0.5*v*sin(Xk_1(3)+w/2));
          (sin(Xk_1(3)+w/2)) (0.5*v*cos(Xk_1(3)+w/2));
           0                     1                                 ]; % G
       
    P_k = Ak*Pk_1*((Ak)') + Bk*Qk_1*((Bk)');

    % Prediccion de la medida (Modelo de observacion)
    Zk_ = Zk;
    Hk = [1 0 0; 0 1 0; 0 0 1];

    % Comparacion
    Yk = Zk-Zk_;
%     for r=1:3
%         if Yk(r)>pi
%             Yk(r) = Yk(r) - 2*pi;
%         end
%         if Yk(r)<(-pi)
%             Yk(r) = Yk(r) + 2*pi;
%         end
%     end
    Sk = Hk*P_k*((Hk)') + Rk;
    Wk = P_k*((Hk)')/Sk;

    % Correccion
    Xk = X_k + Wk*Yk;
    Pk = (eye(3)-Wk*Hk)*P_k;
    
    %Sólo para almacenarlo
    Xestimado(:,k) = Xk;
    Pacumulado(1,k) = Pk(1,1);
    Pacumulado(2,k) = Pk(2,2);
    Pacumulado(3,k) = Pk(3,3);
    
    % Actualizacion del tiempo
    t = t+h;
    apoloUpdate;
end 

% Representacion grafica
figure(1);
axis([0 12 0 9])

plot(Xreal(:,1),Xreal(:,2),'r--')
hold on
plot(Xestimado(:,1),Xestimado(:,2),'b--')
hold off
legend('Movimiento Real','Estimación')



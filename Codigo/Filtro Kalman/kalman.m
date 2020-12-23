% Definimos una trayectoria circular
velocidadL = 0.2;  % Velocidad lineal 0.2 m/seg
timestep = 0.5;  % Actualizacion de sensores
distanciatotal = 4*pi; % en metros.. radio de 2 metros
numerodepasos = ceil(distanciatotal/(velocidadL*timestep));
velocidadA = 2*pi/(numerodepasos*timestep); % Velocidad angular

% Varianza del ruido del proceso 
Qd = 0.01*velocidadL*timestep;
Qb = 0.02*velocidadA*timestep;
Qk_1 = [Qd 0; 0 Qb];

for k = 1:2*numerodepasos
    trayectoriaD(k) = velocidadL*timestep;
    trayectoriaB(k) = velocidadA*timestep;
    trayectoriaDRuido(k) = velocidadL*timestep + sqrt(Qd)*randn;
    trayectoriaBRuido(k) = velocidadA*timestep + sqrt(Qb)*randn;
end

% Inicializamos la posición inicial y su covarianza
xini = 5;
yini = 2;
thetaini = 0;
Xrealk = [xini; yini; thetaini];
Xk = [6; 3; pi];

Pxini = 0.001;
Pyini = 0.001;
Pthetaini = 0.001;
Pk = [Pxini 0 0; 0 Pyini 0 ; 0 0 Pthetaini];

% Varianza en la medida
R1 = 0.001;
R2 = 0.001;
R3 = 0.001;
Rk = [R1 0 0; 0 R2 0; 0 0 R3];

% Posición de las balizas
t1x = 4;
t1y = 8;
t2x = 1;
t2y = 1;
t3x = 11;
t3y = 3;

% Algoritmo
Ktotal = zeros(3);      
for l = 1:length(trayectoriaD)
    % Solo para el simulador
    XrealkAUX = Xrealk;
    
    % Avance real del robot
    Xrealk(1) = XrealkAUX(1) + trayectoriaD(l)*cos(XrealkAUX(3)+(trayectoriaB(l)/2));
    Xrealk(2) = XrealkAUX(2) + trayectoriaD(l)*sin(XrealkAUX(3)+(trayectoriaB(l)/2));
    Xrealk(3) = XrealkAUX(3) + trayectoriaB(l);
    Xreal(:,l) = Xrealk;  % Para mantener una historia del recorrido

    % Observacion de las balizas
    Zk = [(atan2(t1y-Xrealk(2),t1x-Xrealk(1)) - Xrealk(3) + sqrt(R1)*randn);
          (atan2(t2y-Xrealk(2),t2x-Xrealk(1)) - Xrealk(3) + sqrt(R2)*randn);
          (atan2(t3y-Xrealk(2),t3x-Xrealk(1)) - Xrealk(3) + sqrt(R3)*randn)];
          
    % Para acortar el nombre de la variable
    Uk = [trayectoriaDRuido(l); trayectoriaBRuido(l)];

    % Nuevo ciclo, k-1 = k.
    Xk_1 = Xk;
    Pk_1 = Pk;
    
    % Prediccion del estado (Modelo de odometria)
    X_k = [(Xk_1(1) + Uk(1)*cos(Xk_1(3)+(Uk(2)/2)));
           (Xk_1(2) + Uk(1)*sin(Xk_1(3)+(Uk(2)/2)));
           (Xk_1(3) + Uk(2))];
    Ak = [1 0 (-Uk(1)*sin(Xk_1(3)+Uk(2)/2));
          0 1 (Uk(1)*cos(Xk_1(3)+Uk(2)/2));
          0 0 1                             ]; % Fi
    Bk = [(cos(Xk_1(3)+Uk(2)/2)) (-0.5*Uk(1)*sin(Xk_1(3)+Uk(2)/2));
          (sin(Xk_1(3)+Uk(2)/2)) (0.5*Uk(1)*cos(Xk_1(3)+Uk(2)/2));
           0                     1                                 ]; % G
    P_k = Ak*Pk_1*((Ak)') + Bk*Qk_1*((Bk)');

    % Prediccion de la medida (Modelo de observacion)
    Zk_ = [(atan2(t1y-X_k(2),t1x-X_k(1)) - X_k(3));
          (atan2(t2y-X_k(2),t2x-X_k(1)) - X_k(3));
          (atan2(t3y-X_k(2),t3x-X_k(1)) - X_k(3))];
    Hk = [((t1y-X_k(2))/((t1x-X_k(1))^2+(t1y-X_k(2))^2)) (-(t1x-X_k(1))/((t1x-X_k(1))^2+(t1y-X_k(2))^2)) (-1);
          ((t2y-X_k(2))/((t2x-X_k(1))^2+(t2y-X_k(2))^2)) (-(t2x-X_k(1))/((t2x-X_k(1))^2+(t2y-X_k(2))^2)) (-1);   
          ((t3y-X_k(2))/((t3x-X_k(1))^2+(t3y-X_k(2))^2)) (-(t3x-X_k(1))/((t3x-X_k(1))^2+(t3y-X_k(2))^2)) (-1)];

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
    Xestimado(:,l) = Xk;
    Pacumulado(1,l) = Pk(1,1);
    Pacumulado(2,l) = Pk(2,2);
    Pacumulado(3,l) = Pk(3,3);
end 

% Representacion grafica
figure(1);
subplot(2,2,1);
axis([0 12 0 9])
hold on

for m = 1:length(trayectoriaD)
    plot(Xreal(1,m),Xreal(2,m),'.k', Xestimado(1,m),Xestimado(2,m),'.r');
end
legend('Movimiento Real','Estimación')

% Balizas
plot(t1x,t1y,'sk');
plot(t2x,t2y,'sk');
plot(t3x,t3y,'sk');
xlabel ('X (m)')
ylabel ('Y (m)')

subplot(2,2,2);
plot(Xreal(1,:));
hold on
plot(Xestimado(1,:),'g');
xlabel ('t (muestras)')
ylabel ('X (m)')

subplot(2,2,3);
plot(Xreal(2,:));
hold on
plot(Xestimado(2,:),'g');
xlabel ('t (muestras)')
ylabel ('Y (m)')

subplot(2,2,4);
plot(Xreal(3,:));
hold on
plot(Xestimado(3,:),'g');
xlabel ('t (muestras)')
ylabel ('\theta (rad)')
hold off

figure(2);
subplot(3,1,1);
axis([0 12 0 9])
plot(Pacumulado(1,:),'b');
xlabel ('t (muestras)')
ylabel ('Varianza X (m2)')
hold on

subplot(3,1,2);
axis([0 12 0 9])
plot(Pacumulado(2,:),'b');
xlabel ('t (muestras)')
ylabel ('Varianza Y (m2)')

subplot(3,1,3);
axis([0 12 0 9])
plot(Pacumulado(3,:),'b');
xlabel ('t (muestras)')
ylabel ('Varianza \theta (rad2)')

pause
close all
clear all


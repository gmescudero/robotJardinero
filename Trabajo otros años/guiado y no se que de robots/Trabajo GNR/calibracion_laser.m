%% Script para la calibracion de los telemetros laser

%% Inicializacion de variables
% Distancias a probar
distancias = [10 7.5 5 2.5 1]; 
% Angulos a probar (rad)
angulos = [-pi/4 -pi/8 0 pi/8 pi/4];

% Numero de distancias a probar
N = length(distancias);
% Numero de angulos a probar
M = 5;
% Numero de repeticiones de cada ensayo
K = 100;

% Matriz errores
errores = zeros(N,M);
for i = 2:K
    errores(:,:,i) = zeros(N,M);
end
% Matrices para medias y varianzas
mu_e = zeros(N,M);
var_e = zeros(N,M);

% Posicion del laser
x_laser = 1;
y_laser = 24;
theta_laser = -pi/4;

% Arrays para representar
X_estim = zeros(N*M*K);
Y_estim = zeros(N*M*K);
X_real = zeros(N*M*K);
Y_real = zeros(N*M*K);

%% Bucle de comprobación
for i = 1:N
    for j = 1:M
        for k = 1:K
            x = distancias(i)*cos(angulos(j)-theta_laser) + x_laser;
            y = distancias(i)*sin(angulos(j)-theta_laser) + y_laser;
            apoloPlaceMRobot('Giskard', [x y 0], 0);
            apoloUpdate();
            [x_estim, y_estim, ~] = getPoseFromLaser('LMS100A');
            pos = apoloGetLocationRobot('Giskard');
            x_real = pos(1);
            y_real = pos(2);
            errores(i,j,k) = sqrt((x_real-x_estim)^2+(y_real-y_estim)^2);
            X_estim(i+j+k-2) = x_estim;
            Y_estim(i+j+k-2) = y_estim;
            X_real(i+j+k-2) = x_real;
            y_real(i+j+k-2) = y_real;
        end
    end
end

%% Calculo de la media y varianza de cada dist-ang
for i = 1:N
    for j = i:M
        mu_e(i,j) = mean(errores(i,j,:));
        var_e(i,j) = var(errores(i,j,:));
    end
end

plot(X_estim,Y_estim,'x', X_real, Y_real, 'ro');

tabla_mu_e = table(mu_e);
tabla_var_e = table(var_e);
writetable(mu_e, 'mu_e.xlsx');
writetable(var_e, 'var_e.xlsx');

function [Xk,Pk] = getLocation(robot,laser,LM,Xk_1,Pk_1,h,v,w)

%% CONFIG
% Varianza del ruido del proceso 
Qv = 1e-15;
Qw = 1e-16;
% Qk_1 = diag([Qv,Qv,Qw]);
Qk_1 = diag([Qv,Qw]);

% Varianza en la medida
R1 = 0.0136;
R2 = 0.0090;
R3 = 0.0100;

%% RETRIEVE LM
% Se realiza una busqueda de balizas por el laser
baliza = apoloGetLaserLandMarks(laser);
Zk = zeros(3*length(baliza.distance),1);
for j = 1:length(baliza.distance)
    ind = 3*(j-1);
    % Extraccion de las medidas realizadas por el laser
    Zk(ind+1,1) = baliza.distance(j);
    Zk(ind+2,1) = sin(baliza.angle(j));
    Zk(ind+3,1) = cos(baliza.angle(j));
end

%% Prediccion del estado (Modelo de odometria)

% TODO: For some reason the apolo odometry does not work correctly
% Uk = (apoloGetOdometry(robot))';
% apoloResetOdometry(robot);
% 
% Ak = h*eye(3);
% Bk = h*eye(3);

Uk = [v*h*cos(Xk_1(3)+(w*h/2));
      v*h*sin(Xk_1(3)+(w*h/2));
      w*h];

Ak = [  1 0 (-v*h*sin(Xk_1(3)+w*h/2));
        0 1 ( v*h*cos(Xk_1(3)+w*h/2));
        0 0 1                             ]; % Fi
Bk = [  (cos(Xk_1(3)+w*h/2)) (-0.5*v*h*sin(Xk_1(3)+w*h/2));
        (sin(Xk_1(3)+w*h/2)) ( 0.5*v*h*cos(Xk_1(3)+w*h/2));
        0                     1          ]; % G
    
X_k = Xk_1 + Uk;
P_k = Ak*Pk_1*((Ak)') + Bk*Qk_1*((Bk)');


if X_k(3) < -pi
    X_k(3) = X_k(3)+2*pi;
elseif X_k(3) > pi
    X_k(3) = X_k(3)-2*pi;
end

%% Prediccion de la medida (Modelo de observacion)
Zk_     = zeros(3*length(baliza.distance),1);
Rk_aux  = zeros(3*length(baliza.distance),1);
Hk      = zeros(3*length(baliza.distance),3);
Yk      = zeros(3*length(baliza.distance),1);
for j = 1:length(baliza.distance)
    id = baliza.id(j);
    % Calculo de matriz Zk_
    incX = LM(id,1)-X_k(1);
    incY = LM(id,2)-X_k(2);
    incAng = atan2(incY,incX) - X_k(3);
    if incAng < -pi
        incAng = incAng+2*pi;
    elseif incAng > pi
        incAng = incAng-2*pi;
    end
    Zk_(3*j-2,1) = sqrt(incX^2+incY^2);
    Zk_(3*j-1,1) = sin(incAng);
    Zk_(3*j,1) = cos(incAng);
    % Calculo de matriz Hk
    Hk(3*j-2,:) = [incX/sqrt(incX^2+incY^2) incY/sqrt(incX^2+incY^2) 0];
    Hk(3*j-1,:) = [-incY*cos(-incAng)/(incX^2+incY^2) incX*cos(-incAng)/(incX^2+incY^2) -cos(-incAng)];
    Hk(3*j,:) = [incY*sin(-incAng)/(incX^2+incY^2) -incX*sin(incAng)/(incX^2+incY^2) -sin(-incAng)];
    % Calculo de matriz Rk
    Rk_aux(3*j-2)   = R1;
    Rk_aux(3*j-1)   = R2;
    Rk_aux(3*j)     = R3;
    
    % Calculo de matriz Yk
    %% Comparacion
    Yk(3*j-2,1) = -(Zk(3*j-2,1)-Zk_(3*j-2,1));
    Yk(3*j-1,1) = +(Zk(3*j-1,1)-Zk_(3*j-1,1));
    Yk(3*j-0,1) = +(Zk(3*j-0,1)-Zk_(3*j-0,1));
end
Rk = diag(Rk_aux);
% ----------------------------------------------------------------

%% Comparacion
Sk = Hk*P_k*((Hk)') + Rk;

%% Correccion
Wk = P_k*((Hk)')/Sk;
Xk = X_k + Wk*Yk;
Pk = (eye(3) - Wk*Hk)*P_k;

% Correcion de angulo
if Xk(3) < -pi
    Xk(3) = Xk(3)+2*pi;
elseif Xk(3) > pi
    Xk(3) = Xk(3)-2*pi;
end

end


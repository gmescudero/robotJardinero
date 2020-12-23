% ------------ Calibracion sensor ------------
% Media del error: [0.0009544, -0.0034936, 1.5707]
% Varianza del error: [0.0014925, 0.0021356, 0.00048972]
% --------------------------------------------
clear

%% Inicializamos
% Inicializamos la posición inicial
xini = 1.0;
yini = 1.0;
thetaini = 0;
Xrealk = [xini; yini; thetaini];
Xk = Xrealk;

% Inicializamos covarianzas
Pxini = 0.001;
Pyini = 0.001;
Pthetaini = 0.001;
Pk = [Pxini 0 0; 0 Pyini 0 ; 0 0 Pthetaini];

% Inicializamos varianzas de medidas
% TODO check this
R1 = 0.001;
R2 = 0.001;
R3 = 0.001;
Rk = [R1 0 0; 0 R2 0; 0 0 R3];

% Posiciones de balizas
LM = [... % LM(lmID,[X,Y,Z])
    [10.35  5.55    0.2];
    [0      0       0.2];
    [0      13.5	0.2];
    [26.5	13.5	0.2];
    [26.5	0       0.2];
    [9.2	6.75	0.2];
    [11.3	6.75	0.2];
    [10.4	8       0.2];
    [15.7	9.9     0.2];
    [15.5	3.3     0.2];
    [20.5	7.3     0.2];
    [20.5	6.1		0.2];
    [20.3	12.2	0.2];
    [20.3	1.1     0.2];
    [24     5.6     0.2];
    [23.6	7.6     0.2];
    [22.3	8.4     0.2];
    [22.5	5.5     0.2];
    [11.3	0.7     0.2];
    [11.4	12.4	0.2];
    [5.8	10.1	0.2];
    [5.9	4.0     0.2];
    [1.4	7.2     0.2];
];

%% Algoritmo
% Retrieve measures
[X_kp1,P_kp1] = kalmanFilter(x_k,P_k,U_kp1,Z_kp1);






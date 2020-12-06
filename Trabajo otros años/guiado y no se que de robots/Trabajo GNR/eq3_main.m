%% Script principal para la ejecuccion de la simulacion de GNR
% Autores:
% Montserrat Díaz-Carrasco Díaz 14122
% José Luis Millán Valbuena     14302
% Jesús Pareja Muñoz            14335

clear;

%% Inicializacion de variables
global piezasA piezasB piezasC piezasD;
global t
t = 0;
h = 1;

%% Bucle principal
while 1
    % Llama al script de generar y elinimar piezas
    generar_piezas
    
    if piezasA > piezasB
        % ir a A
    else
        % ir a B
    end
    
    if piezasC > piezasD
        % ir a C
    else
        % ir a D
    end
    
    t = t + h;
    % pause(0.01);
end
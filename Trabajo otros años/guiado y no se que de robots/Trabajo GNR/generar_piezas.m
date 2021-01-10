%% Generación y eliminación de las piezas
% Codigo para llamar desde el main

%% Inicializacion de variables
global piezasA piezasB piezasC piezasD
global t

%% Primera llamada
% En el instante t0, se pone todo a su valor inicial, y se definen las
% medias de generacion y eliminacion de cada pieza
if t == 0
    piezasA = 0;
    piezasB = 0;
    piezasC = 100;
    piezasD = 100;
    lambdaA = 3;
    lambdaB = 3;
    lambdaC = 3;
    lambdaD = 3;
    
    tA = t + exponencial(lambdaA);
    tB = t + exponencial(lambdaB);
    tC = t + exponencial(lambdaC);
    tD = t + exponencial(lambdaD);
%% Bucle
% Para cada instante siguiente, se hace esta parte
else
    % Si el tiempo es mayor al de generacion de A
    if t>tA
        % Hay menos de 100 piezas
        if piezasA < 100
            % Se suma una pieza
            piezasA = piezasA + 1;
            % Se genera el siguiente evento
            tA = t + exponencial(lambdaA);
        end
    end
    if t>tB
        if piezasB < 100
            piezasB = piezasB + 1;
            tB = t + exponencial(lambdaB);
        end
    end
    if t>tC
        if piezasC>0
            piezasC = piezasC - 1;
            tC = t + exponencial(lambdaC);
        end
    end
    if t>tD
        if piezasD > 0
            piezasD = piezasD - 1;
            tD = t + exponencial(lambdaD);
        end
    end
    % Esto es para mostrar las piezas (prescindible)
    piezas = [piezasA piezasB piezasC piezasD];
    x = categorical({'A','B','C','D'});
    bar(x, piezas);
    ylim([0 100]);
    pause(0.01);
end

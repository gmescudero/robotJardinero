%% Generacion y eliminacion aleatoria de piezas en 4 puntos
% Codigo de prueba para la generacion de piezas

clear;


piezasA = 0;
piezasB = 0;
piezasC = 100;
piezasD = 100;

t = 0;
h = 1;

lambdaA = 3;
lambdaB = 3;
lambdaC = 3;
lambdaD = 3;

tA = t + exponencial(lambdaA);
tB = t + exponencial(lambdaB);
tC = t + exponencial(lambdaC);
tD = t + exponencial(lambdaD);
pause();
while 1
    
    if t>tA
        if piezasA < 100
            piezasA = piezasA + 1;
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
    t = t + h;
    piezas = [piezasA piezasB piezasC piezasD];
    x = categorical({'A','B','C','D'});
    bar(x, piezas);
    ylim([0 100]);
    pause(0.01);
    
    if (piezas == [100 100 0 0])
        break
    end
end
disp('End of simulation');
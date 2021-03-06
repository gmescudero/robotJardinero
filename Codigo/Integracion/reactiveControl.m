function [vf, vg,tooClose] = reactiveControl(...
    dir)    % Prefered turning dir: 1 izda, -1 dcha
tooClose = false;

%% Config
lim1 = 0.60;
lim2 = 0.40;
lim3 = 0.25;

%% Agorithm
frente      = min([ apoloGetUltrasonicSensor('uc'),...
                    apoloGetUltrasonicSensor('uci'),...
                    apoloGetUltrasonicSensor('ucd')]);
izquierda   = min([ apoloGetUltrasonicSensor('ul'),...
                    apoloGetUltrasonicSensor('uci')]);
derecha     = min([ apoloGetUltrasonicSensor('ur'),...
                    apoloGetUltrasonicSensor('ucd')]);

if frente > lim1 && izquierda > lim2 && derecha > lim2
    % No near obstacles
    vf = 1;
    vg = 0;
elseif frente > lim2 && izquierda > lim2 && derecha > lim2
    % Obstable not close
    vf = 0.4;
    vg = 1.0;
    if dir == 1
        if izquierda < frente
            vg = -vg;
        end
    else
        if derecha > frente
            vg = -vg;
        end
    end
elseif frente > lim3 && izquierda > lim3 && derecha > lim3
    % Obstable close
    vf = 0.0;
    vg = 1.2;
    if dir == 1
        if izquierda < frente
            vg = -vg;
        end
    else
        if derecha > frente
            vg = -vg;
        end
    end
else
    % Obstacle too close
    tooClose = true;
    vf = 0;
    vg = dir;
end

end


function [vf, vg,tooClose] = reactiveControl(...
    dir)    % Prefered turning dir (-1,1)
tooClose = false;

%% Config
lim1 = 0.7;
lim2 = 0.25;

%% Agorithm
frente      = min([apoloGetUltrasonicSensor('uci'),apoloGetUltrasonicSensor('ucd')]);
izquierda   = apoloGetUltrasonicSensor('ul');
derecha     = apoloGetUltrasonicSensor('ur');

if frente > lim1 && izquierda > lim2 && derecha > lim2
    % No near obstacles
    vf = 1;
    vg = 0;
elseif frente > lim2 && izquierda > lim2 && derecha > lim2
    % Obstable close
    vf = 0.5;
    vg = 0.5;
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


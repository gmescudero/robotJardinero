function [vf, vg,tooClose] = reactiveControl()
tooClose = false;

%% Config
lim1 = 0.6;
lim2 = 0.3;

%% Agorithm
frente      = apoloGetUltrasonicSensor('uc0');
izquierda   = apoloGetUltrasonicSensor('ul1');
derecha     = apoloGetUltrasonicSensor('ur1');

if frente > lim1 && izquierda > lim2 && derecha > lim2
    % No near obstacles
    vf = 1;
    vg = 0;
elseif frente > lim2 && izquierda > lim2 && derecha > lim2
    tooClose = true;
    % Obstable close
    vf = 0.5;
    if izquierda > lim2
        vg = 1;
    else
        vg = -1;
    end
else
    % Obstacle too close
    tooClose = true;
    vf = 0;
    if frente < izquierda
        vg = 1;
    else
        vg = -1;
    end
end

end


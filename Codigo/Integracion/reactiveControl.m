function [vf, vg] = reactiveControl()

frente      = apoloGetUltrasonicSensor('uc0');
izquierda   = apoloGetUltrasonicSensor('ul1');
derecha     = apoloGetUltrasonicSensor('ur1');

%% Agorithm
lim1 = 0.8;
lim2 = 0.4;

if frente > lim1 && izquierda > lim2 && derecha > lim2
    % No near obstacles
    vf = 1;
    vg = 0;
elseif frente > lim2
    % Obstable close
    vf = 0.5;
    if izquierda > lim2
        vg = 1;
    else
        vg = -1;
    end
else
    % Obstacle too close
    vf = 0;
    if frente < izquierda
        vg = 1;
    else
        vg = -1;
    end
end

end


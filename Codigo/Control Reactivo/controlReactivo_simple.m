clearvars

%% Compute robot C space
% q1 = -1.07:0.01:0.44;
% q2 = -0.21:0.01:0.77;
% robot = 'ASEA';
% a = cspace(robot,q1,q2);
% 
% %% General operations
% robot = 'Pioneer3AT';
% apoloPlaceMRobot(robot,[0,0,pi/2],0)
% apoloGetLocation(robot) % returns location

%% Control reactivo
robot = 'Marvin';

apoloPlaceMRobot(robot,[1,1,0],0)
apoloUpdate()

while true
    frente      = apoloGetUltrasonicSensor('uc0');
    izquierda   = apoloGetUltrasonicSensor('ul1');
    derecha     = apoloGetUltrasonicSensor('ur1');
    
    vf = 0; vg = 0;
    lim1 = 0.8;
    lim2 = 0.4;
    
    if frente > lim1 && izquierda > lim2 && derecha > lim2
        vf = 1;
    elseif frente > lim2
        vf = 0.5;
        if izquierda > lim2
            vg = 1;
        else
            vg = -1;
        end
    else
        if frente < izquierda
            vg = 1;
        else
            vg = -1;
        end
    end
    ret = apoloMoveMRobot(robot,[vf vg],0.1);

    apoloUpdate()
    pause(0.1);
end
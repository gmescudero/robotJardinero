clearvars

%% Compute robot C space
% q1 = -1.07:0.01:0.44;
% q2 = -0.21:0.01:0.77;
% robot = 'ASEA';
% a = cspace(robot,q1,q2);
% 
% %% General operations
% robot = 'Pioneer3AT';
% apoloPlaceMRobot(robot,[3,13,0],0)
% apoloGetLocation(robot) % returns location

%% Control reactivo
robot = 'Marvin';

while true
    frente    = apoloGetUltrasonicSensor('uc0');
    izquierda = apoloGetUltrasonicSensor('ul1');
    vf = 0; vg = 0;
    lim = 0.4;
    if frente > lim
        vf = 1;
    else
        if frente < izquierda
            vg = 1;
        else
            vg = -1;
        end
    end
    ret = apoloMoveMRobot(robot,[vf vg],0.1);
    if 0 == ret
        vg = 0.2;
        vf = -0.5;
        ret = apoloMoveMRobot(robot,[vf vg],0.1);
        if 0 == ret
            vg = 1;
            vf = 0;
            apoloMoveMRobot(robot,[vf vg],0.1);
        end
    end
    apoloUpdate()
    pause(0.1);
end
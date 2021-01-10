
clearvars

%% Control reactivo
robot = 'Marvin';
laser = 'LMS100';
sampleT = 0.1;
u = zeros(3,1);

vMax = 1.0;
wMax = 1.0;
vf = vMax;
vg = 0;

apoloPlaceMRobot(robot,[1,1,0],0)
apoloUpdate()

way = [8.8,7];

Kf = 1;
Kg = 1;

ret = 1;
while ret ~= 0 && (vf > 0.1 || vg > 0.1)
    %% Measures (TODO, use kalman and real measures)
    XrealAUX = apoloGetLocationMRobot(robot);
    
    Xreal(1) = XrealAUX(1);
    Xreal(2) = XrealAUX(2);
    Xreal(3) = XrealAUX(4);
    
    %% Trajectory   
    % Compute velocities
    [vf, vg] = trajectoryControlP(way,Xreal,Kf,Kg, vMax ,wMax);
    
    %% Reactive
    % Get ultrasonic measurements
    u(1) = apoloGetUltrasonicSensor('uc0');
    u(2) = apoloGetUltrasonicSensor('ul1');
    u(3) = apoloGetUltrasonicSensor('ur1');
    
    % Compute velocities based on obstacles distance
    [reactVf, reactVg] = reactiveControl(u);
    if reactVf < 0.9 && vf > 0.1
        vf = reactVf;
        vg = reactVg;
    end
    
    %% Move
    % Use retrieved velocities to command the robot
    ret = apoloMoveMRobot(robot,[vf vg],sampleT);
    
    % Update Apolo info and synchronizate movement
    apoloUpdate()
    pause(sampleT);
end
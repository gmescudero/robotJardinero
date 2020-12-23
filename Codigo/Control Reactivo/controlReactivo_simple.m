clearvars

%% Control reactivo
robot = 'Marvin';
laser = 'LMS100';
sampleT = 0.1;
u = zeros(3,1);

apoloPlaceMRobot(robot,[1,1,0],0)
apoloUpdate()

while true
    % Get ultrasonic measurements
    u(1) = apoloGetUltrasonicSensor('uc0');
    u(2) = apoloGetUltrasonicSensor('ul1');
    u(3) = apoloGetUltrasonicSensor('ur1');
    
    % Compute velocities based on obstacles distance
    [vf, vg] = reactiveControl(u);
    
    % Use retrieved velocities to command the robot
    ret = apoloMoveMRobot(robot,[vf vg],sampleT);
    
    % Update Apolo info and synchronizate movement
    apoloUpdate()
    pause(sampleT);
end
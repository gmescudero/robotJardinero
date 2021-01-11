clearvars

% Posicion robot
robot.name = 'Marvin';
robot.pos = [0, 0, 0];
robot.ang = [pi/2];
% Posicion del laser respecto al robot
laser.pos(1) = 0.1;
laser.pos(2) = 0;
laser.ang = 0;
apoloPlaceMRobot(robot.name, robot.pos, robot.ang);

v = 1;
w = 1;
h = 0.1;
times = 10;

Nsim = 10000;
Vodo = zeros(Nsim,2);
X_k  = zeros(Nsim,3);

for j = 1:Nsim
    apoloPlaceMRobot(robot.name, robot.pos, robot.ang);
    apoloUpdate();
    apoloResetOdometry(robot.name);
    for i = 1:times
        ret = apoloMoveMRobot(robot.name, [v w], h);
        apoloUpdate();
    end

    Xodo = apoloGetOdometry(robot.name);
    Vodo(j,1) = Xodo(1);
    Vodo(j,2) = Xodo(2);
    Vodo(j,3) = sqrt(Xodo(1)^2 + Xodo(2)^2)/(h*times);
    Vodo(j,4) = Xodo(3)/(h*times);
    
    X_k(j,:) = [robot.pos(1) + Xodo(1)*cos(robot.ang) - Xodo(2)*sin(robot.ang),...
                robot.pos(2) + Xodo(1)*sin(robot.ang) + Xodo(2)*cos(robot.ang),...
                robot.ang + Xodo(3)];
    
    pause(0.001);
end

XrealAUX = apoloGetLocationMRobot(robot.name);

disp(['REAL [X,Y,Theta,V,W]: [',...
    num2str(XrealAUX(1)),', ',...
    num2str(XrealAUX(2)),', ',...
    num2str(XrealAUX(4)),', ',...
    num2str(v),', ',...
    num2str(w) ,']'])

disp(['X:    mean: ', num2str(mean(Vodo(:,1))), ', std: ',  num2str(std(Vodo(:,1)))])
disp(['Y:    mean: ', num2str(mean(Vodo(:,2))), ', std: ',  num2str(std(Vodo(:,2)))])
disp(['V:    mean: ', num2str(mean(Vodo(:,3))), ', std: ',  num2str(std(Vodo(:,3)))])
disp(['W:    mean: ', num2str(mean(Vodo(:,4))), ', std: ',  num2str(std(Vodo(:,4)))])

disp(['X(1): mean: ', num2str(mean(X_k(:,1))), ', std: ',  num2str(std(X_k(:,1)))])
disp(['X(2): mean: ', num2str(mean(X_k(:,2))), ', std: ',  num2str(std(X_k(:,2)))])
disp(['X(3): mean: ', num2str(mean(X_k(:,3))), ', std: ',  num2str(std(X_k(:,3)))])


%% results
% REAL [X,Y,V,W]: [ 0.81778 0.50139 1 1 ]
% X: mean: 0.81767, std: 0.014335
% Y: mean: 0.50138, std: 0.009588
% V: mean: 0.95916, std: 0.016541
% W: mean: 0.99997, std: 0.0064275

% REAL [X,Y,V,W]: [ 1 0 1 0 ]
% X: mean: 0.99992, std: 0.017064
% Y: mean: 0, std: 0
% V: mean: 0.99992, std: 0.017064
% W: mean: 0, std: 0

% REAL [X,Y,V,W]: [0, 1, 1, 0]
% X: mean: 1.0004, std: 0.017367
% Y: mean: 0, std: 0
% V: mean: 1.0004, std: 0.017367
% W: mean: 0, std: 0

%% Conclusion
% The X and Y values are calculated from the last X Y axes of the robot so
% we have to adapt to that


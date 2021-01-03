% Waypoints 
camino = [... wp(xPos,yPos)
%     [ 2  2];
%     [-2  2];
%     [-2 -2];
%     [ 2 -2];
    [    1.0000    1.0000];
    [    5.0055    2.9653];
    [   13.5643   13.2041];
    [   24.3775   14.1838];
    [   31.4076   20.3689];
    [   41.4418   29.5726];
    [   46.8712   26.3391];
    [   55.1523   26.5495];
    [   64.2395   29.6187];
    [   74.0155   30.8810];
    [   86.6054   31.9431];
    [  101.4533   33.1906];
    [  112.2420   37.1852];
    [  120.4202   35.0337];
    [  122.5935   27.6108];
    [  120.0000   30.0000];
];
wp = camino.*[(26.5/125) (13.5/65)]; % adjust limits

% Posicion robot
robot.name = 'Marvin';

robot.pos  = [wp(1,:), 0];
robot.ang  = 0;
ret = ones(length(wp(:,1)),1);
for i = 1:length(wp(:,1))
    apoloPlaceMRobot(robot.name, [wp(i,:),0], robot.ang);
    ret(i) = apoloMoveMRobot(robot.name, [0.01 0.01], 0.01);
    apoloUpdate();
    
%     pause
end
ret

load jardinBinMap2.mat

[yRate,xRate] = size(BW);
xRate = xRate/125;
yRate = yRate/65;

imshow (BW)
hold on
for i = 1:length(wp(:,1))
    x = camino(i,1)*xRate;
    y = camino(i,2)*yRate;
    plot(x,y, 'o');
end

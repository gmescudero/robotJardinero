
mapSize.scale = 50;
mapSize.x = round(26.5*mapSize.scale);
mapSize.y = round(13.5*mapSize.scale);

BW = zeros(mapSize.x,mapSize.y);
robotName = 'Marvin';

for x = 1:mapSize.x
    for y = 1:mapSize.y
        apoloPlaceMRobot(robotName, [x/mapSize.scale,y/mapSize.scale,0], 0);
        bin = apoloMoveMRobot(robotName, [0.01 0.01], 0.01);
        if 0 == bin && x > 1 && y > 1
            BW(x-1:x+1,y-1:y+1) = false;
        else
            BW(x,y) = true;
        end
        apoloUpdate();
    end
end
for y = 1:mapSize.y
    for x = 1:mapSize.x
        apoloPlaceMRobot(robotName, [x/mapSize.scale,y/mapSize.scale,0], 0);
        bin = apoloMoveMRobot(robotName, [0.01 0.01], 0.01);
        if 0 == bin && x > 1 && y > 1
            BW(x-1:x+1,y-1:y+1) = false;
        else
            BW(x,y) = true;
        end
        apoloUpdate();
    end
end

imshow(BW);
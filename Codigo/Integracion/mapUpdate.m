function [BW] = mapUpdate(laserName,BW,scale,X)

%% Config
maxDist = 1;
minDist = 0.15;
%% Funct
ld = apoloGetLaserData(laserName);

[ySize,xSize] = size(BW);
newObs = zeros(xSize,ySize) ~= 0;

for i = 1:length(ld)
    angl = (i/length(ld))*3*pi/2 - 3*pi/4 + X(3);
    dist = ld(i);
    if dist < maxDist && dist > minDist
        % Rellenamos un pixel encontrado
        indx1 = min([xSize,max([1,     round((X(1)+cos(angl)*dist)*scale)])]);
        indy1 = min([ySize,max([1,     round((X(2)+sin(angl)*dist)*scale)])]);
        
        indx2 = min([xSize,max([1,     round((X(1)+cos(angl)*(dist+0.01))*scale)])]);
        indy2 = min([ySize,max([1,     round((X(1)+sin(angl)*(dist+0.01))*scale)])]);
        
        if ~BW(indy1,indx1)
            newObs(indx1,indy1) = true;
            newObs(indx2,indy2) = true;
        end
    end
end

newObs = imrotate(newObs,90);

BW = BW | newObs(1:ySize,1:xSize);

end


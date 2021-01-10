function [Zest,H] = pos2measures(...
    X)  % [] Position

% Land Marks positions
global LM

[lmNum,~] = size(LM);
Zest = zeros(lmNum*3,1);
H = zeros(3,lmNum*3);

for i = 1:lmNum
    ind = (i-1)*3; % index
    xGap = (LM(i,1) - X(1));
    yGap = (LM(i,2) - X(2));
    
%% Compute estimated measure
    Zest(ind+1) = i;
    Zest(ind+2) = sqrt(xGap^2 + yGap^2);
    Zest(ind+3) = atan2(yGap,xGap) - X(3);
    
%% Compute H matrix TODO
    % x 
    H(1,ind+1) = 0;
    H(1,ind+2) = xGap/sqrt(xGap^2 + yGap^2);
    H(1,ind+3) = asec(yGap/xGap)^2 * (-yGap/(xGap^2));
    % y 
    H(2,ind+1) = 0;
    H(2,ind+2) = yGap/sqrt(xGap^2 + yGap^2);
    H(2,ind+3) = asec(yGap/xGap)^2/xGap;
    % theta
    H(3,ind+1) = 0;
    H(3,ind+2) = 0;
    H(3,ind+3) = -1;
end
end


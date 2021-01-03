function [ret,path] = mappingAndPlan(...
    type,...        % RRT(1) or PRM(2)
    map,...         % Binary ocupancy map
    Xinit,...       % Initial position
    Xgoal,...       % Goal position
    cellsPerMeter)  % Pixels per meter

RRT = 1;
PRM = 2;

%% Config
% General
robotRadius = 0.3;

% RRT
valDist         = 0.6;
maxIterations   = 10000;
maxConnectDist  = 0.5;

% PRM 
numNodes    = 40;
maxNumNodes = 10000;
conDist     = 4;

%% Map treatment
mapa = binaryOccupancyMap(map, cellsPerMeter);

mapInflated = copy(mapa);
inflate(mapInflated, robotRadius);
% show(mapInflated);

if RRT == type
    %% RRT*
    ss = stateSpaceDubins; %generamos espacio de estados
    %stateSpaceSE2
    %stateSpaceDubins
    %stateSpaceReedsShepp
    sv = validatorOccupancyMap(ss); %validador de ocupancia
    
    sv.Map = mapInflated;
    sv.ValidationDistance = valDist; %Distancia válida entre nodos
    ss.StateBounds = [mapInflated.XWorldLimits; mapInflated.YWorldLimits; [-pi pi]];
    
    planificador = plannerRRTStar(ss,sv);
    planificador.ContinueAfterGoalReached = false;
    planificador.MaxIterations = maxIterations;
    planificador.MaxConnectionDistance = maxConnectDist;

    %Definimos nodo inicial y nodo final
    initPos = [Xinit(1),Xinit(2),0];
    goalPos = [Xgoal(1),Xgoal(2),0];
    [pthObj, solnInfo] = plan(planificador, initPos, goalPos);
    ret = solnInfo.IsPathFound;
    if ret
%         pthObj.interpolate(80);
        path = zeros(pthObj.NumStates,2);
        path(:,:) = [pthObj.States(:,1) pthObj.States(:,2)];
    else
        disp('RRT: Path not found');
        path = 0;
        return
    end

elseif PRM == type
    %% PRM
    prm = mobileRobotPRM;
    prm.Map = mapInflated;
    prm.NumNodes = numNodes;
    prm.ConnectionDistance = conDist;

    initPos = [Xinit(1),Xinit(2)];
    goalPos = [Xgoal(1),Xgoal(2)];
    
    path = findpath(prm, initPos, goalPos);
    while isempty(path) && prm.NumNodes <= maxNumNodes
        prm.NumNodes = prm.NumNodes + 10;
        update(prm);
        path = findpath(prm, initPos, goalPos);
    end
    if prm.NumNodes > maxNumNodes
        disp('PRM: Path not found');
        ret = false;
        path = 0;
        return
    else
        ret = true;
    end
else
    %% Invalid algorithm selection
    disp('Invalid planning algorithm');
    path = 0;
    return
end
end


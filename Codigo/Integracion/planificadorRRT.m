
%% PLANIFICADOR

clearvars; clc;

%RRT*
ss = stateSpaceDubins; %generamos espacio de estados
%stateSpaceSE2
%stateSpaceDubins
%stateSpaceReedsShepp

sv = validatorOccupancyMap(ss); %validador de ocupancia

load jardinBinMap2.mat

mapa = occupancyMap( BW, 10); %10 celdas cada metro

robotRadius = 0.3; %se incrementan los arbustos por el radio de Marvin
mapInflated = copy(mapa); 
inflate(mapInflated, robotRadius);
show(mapInflated);

sv.Map = mapInflated;

sv.ValidationDistance = 0.6; %Distancia válida entre nodos

ss.StateBounds = [mapInflated.XWorldLimits; mapInflated.YWorldLimits; [-pi pi]];

planificador = plannerRRTStar(ss,sv);
planificador.ContinueAfterGoalReached = true;

planificador.MaxIterations = 6000;
planificador.MaxConnectionDistance = 0.5;



%Definimos nodo inicial y nodo final
posini = [ 1, 1, 0];
meta = [ 120, 30, 0];

robot.ang = [pi/2];

rng(100, 'twister');
[pthObj, solnInfo] = plan(planificador,posini,meta);


mapa.show;

hold on;
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2), '.-'); %expansión del arbol
plot(pthObj.States(:,1),pthObj.States(:,2), 'r-','LineWidth',2); %dibujar el camino
title('RRT*');

%% PRM
clearvars; clc;

load jardinBinMap2.mat

whos *Map*

mapa = binaryOccupancyMap(BW, 10); %diez metros por celda (escala entre 10)
figure;
show(mapa);

robotRadius = 0.3; %se incrementan los arbustos por el radio de Marvin
mapInflated = copy(mapa); 
inflate(mapInflated, robotRadius);
show(mapInflated);

prm = mobileRobotPRM;

prm.Map = mapInflated;
prm.NumNodes = 40;
prm.ConnectionDistance = 15;
show(prm);

posini = [ 1, 1];
meta = [ 120, 30];

camino = findpath(prm, posini, meta);

while isempty(camino)
    prm.NumNodes = prm.NumNodes + 10;
    update(prm);
    camino = findpath(prm, posini, meta);
end
camino %Nos da el camino que sigue

show(prm);
title('PRM');

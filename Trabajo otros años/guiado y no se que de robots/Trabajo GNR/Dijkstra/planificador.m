%Script para realizar la planificacion a traves de los diversos waypoints
%tambien conocida como path planning.

%Definimos, en primer lugar, una matriz que recoge los diferentes waypoints
%y su posicion en el plano
global N;
N = 12; %Numero de waypoints del grafo
global G;
G = zeros(N,3); %Inicializamos el grafo
G(1,:)=['A' 3 23];
G(2,:)=['B' 3 1];
G(3,:)=[1 11 19.5];
G(4,:)=[2 11 6];
G(5,:)=[3 21.5 19.5];
G(6,:)=[4 21.5 6];
G(7,:)=[5 31.5 19.5];
G(8,:)=[6 31.5 6];
G(9,:)=[7 37.5 19.5];
G(10,:)=[8 37.5 3];
G(11,:)=['C' 50 23];
G(12,:)=['D' 50 1];

%ALGORITMO DE DIJKSTRA

%Asignamos a cada nodo una distancia tentativa. Esta dist valdra
%0 para el nodo inicial e infinito (9999) para el resto de nodos
global distancias;
distancias = zeros(N,2);
%El vector distancias tiene 2 columnas, la primera es el identif
%del nodo, la segunda la distancia
distancias(:,1) = G(:,1);
for i=1:N
    distancias(i,2) = 9999;
end
%Ponemos la distancia del nodo inicial a 0 y añadimos el nodo
%inicial a la lista de nodos
n0='C';
for i=1:N
    if distancias(i,1) == n0
        distancias(i,2) = 0;
    end
end
ngoal='D';
q=dijkstra(n0,ngoal);
disp("La lista de pasos es:");
fprintf('\n');
disp("q =");
fprintf('\n');
disp(q+0);





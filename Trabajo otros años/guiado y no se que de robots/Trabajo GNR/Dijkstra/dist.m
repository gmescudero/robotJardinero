function d = dist(n1, n2)
%Esta funcion calcula la distancia en x, en y, y la euclidea entre
%dos nodos cualesquiera del grafo.

%n1 y n2 son los identificadores de los nodos del grafo
global N;
global G;
%Creamos dos variables auxiliares y les asignamos las posiciones 
%de los nodos en x1 y x2
x1 = [0 0];
x2 = [0 0];
for i=1:N
    if G(i,1)==n1
        x1(1) = G(i,2);
        x1(2) = G(i,3);
    end
    if G(i,1)==n2
        x2(1) = G(i,2);
        x2(2) = G(i,3);
    end
    i=i+1;
end
%Calculamos las 3 distancias
dx = abs(x1(1)-x2(1));
dy = abs(x1(2)-x2(2));
d = sqrt(dx*dx+dy*dy);
end


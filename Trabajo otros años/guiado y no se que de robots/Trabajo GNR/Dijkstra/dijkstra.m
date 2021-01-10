function q = dijkstra(n0,ngoal)
%Resuelve la parte iterativa del algoritmo de dijkstra.
%Devuelve en q la secuencia de waypoints necesaria. Ademas, modifica
%la matriz distancias con las distancias mas cortas
global N;
global G;
global distancias;
q=n0;
%n es el nodo actual. n0 es el nodo inicial
n=n0;

while n~=ngoal
    %Mostramos la lista q, el +0 es para mostrar los numeros
    disp(q+0);
    %Si q es demasiado grande (demasiados pasos) significa que algo
    %ha fallado, asi que, salimos del bucle y de la funcion con
    %un mensaje de error
    if length(q)>100
        disp("ERROR 01: Excedido tamaño de q");
        disp("Abortando algoritmo");
        return;
    end
    %Hallamos los vecinos del nodo y guardamos en a cuantos vecinos tiene
    qv=vecinos(n);
    a=length(qv);
    
    %Exploramos en este bucle todos los nodos vecinos no visitados
    for i=1:a
        %Comprobamos si el nodo esta ya en q (ya visitado) o no
        %qv(i) es el nodo vecino que se evalua en cada vuelta
        nodo_visitado = 0;
        for j=1:length(q)
            if qv(i)==q(j)
                nodo_visitado=1;
            end
        end
        %Si no ha sido visitado, entonces:
        if ~nodo_visitado
            %Calculamos la distancia total del nodo evaluado
            d=dist(qv(i),n)+dist(n,n0);
            %Recogemos en j la fila del nodo evaluado en distancias
            for j=1:N
                if distancias(j,1) == qv(i)
                    break;
                end
            end
            %Comparamos la distancia tentativa con la dist real (d)
            if d < distancias(j,2)
                distancias(j,2) = d;
            end
        end
    end
    %Marcamos n como visitado, es decir, lo añadimos a la lista q
    q = [q n];
    %Comprobamos si se ha añadido el nodo objetivo
    if n==ngoal
        break;
    end
    
    %Ahora elegimos como siguiente "nodo actual" al nodo con la
    %distancia tentativa (distancias) mas pequeña, DE LOS QUE NO
    %ESTEN YA VISITADOS
    
    %Tomamos, por ejemplo, el primer nodo vecino NO VISITADO
    %(No tiene por que ser el primer nodo vecino de qv).
    for i=1:a
        n=qv(i);
        nodo_visitado = 0;
        for j=1:length(q)
            if n==q(j)+0
                nodo_visitado=1;
                break;
            end
        end
        if ~nodo_visitado
            %Elegimos su fila mediante el identificador y guardamos en d
            %su distancia tentativa
            for j=1:N
                if distancias(j,1) == qv(i)
                    break;
                end
            end
            break;
        end
    end
    d=distancias(j,2);
    %Evaluamos el resto de nodos vecinos para ver si su distancia
    %tentativa es menor
    while i<=a
        %Comprobamos si el nodo esta ya en q (ya visitado) o no.
        %qv(i) es el nodo vecino que se evalua en cada vuelta
        nodo_visitado = 0;
        for j=1:length(q)
            if qv(i)==q(j)
                nodo_visitado=1;
            end
        end
        if ~nodo_visitado
            %Recogemos en j la fila del nodo evaluado en distancias
            for j=1:N
                if distancias(j,1) == qv(i)
                    break;
                end
            end
            %Si la distancia tentativa del nodo evaluado es menor que la
            %actual, entonces actualizamos el nodo actual y su distancia
            if distancias(j,2)<d
                n=qv(i)
                d=distancias(j,2);
            end
        end
        i=i+1;
    end
    
    %Fin Algoritmo
end
%Añadimos el nodo objetivo a la lista
q=[q ngoal];
end


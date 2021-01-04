function qvecinos = vecinos(n)
%Devuelve la lista de los nodos vecinos al nodo n
%Dicho de otra forma, Informa de las relaciones del grafo
global N;
global G;
qvecinos = n;
switch n
    case 'A'
        qvecinos=1;
    case 'B'
        qvecinos=2;
    case 1
        qvecinos=['A' 2 3];
    case 2
        qvecinos=['B' 1 4];
    case 3
        qvecinos=[1 4 5];
    case 4
        qvecinos=[2 3 6];
    case 5
        qvecinos=[3 6 7];
    case 6
        qvecinos=[4 5];
    case 7
        qvecinos=[5 8 'C'];
    case 8
        qvecinos=[7 'D'];
    case 'C'
        qvecinos=7;
    case 'D'
        qvecinos=8;
end
end


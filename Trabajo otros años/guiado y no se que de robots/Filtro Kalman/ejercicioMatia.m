R = 5e-6;
x = [0.5; 0.6];
P = [5 0; 0 5]*1e-6;
z_10 = [0.602; 0.496; 0.599; 0.503; 0.598; 0.601; 0.497; 0.598; 0.501; 0.600; 1.0; 0.5; 1.1; 0.492];

n = 1;
while n<length(z_10)+1
    if (mod(n,2)~=0)
        H = [0 1];
    else
        H = [1 0];
    end
    
    S = H*P*H'+R;
    W = P*H'/S;
    P = P-W*H*P;
    v = z_10(n)-H*x;
    x = x + W*v;
    
    disp("x(" +num2str(n) + ") = " + num2str(x'));
    disp("P(" +num2str(n) + ") = " + num2str(P));
   
    n = n+1;
end

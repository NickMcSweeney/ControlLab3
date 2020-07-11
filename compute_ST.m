function [S, T] = compute_ST(y_refs, u_refs, dt, A_, a_, gamma_) 
% Compute S, T using the reference y and u in the vector (here we assume
% that the length of the refs is our preview window).

D = length(y_refs);
D2 = length(u_refs);

if (D ~= D2) 
    if (D > D2)
        D = D2;
    end
end

S = zeros(D, 1);
T = zeros(D, D);

for i = 1:D
    Ai = compute_A(y_refs(i), u_refs(i), dt, A_, a_);
    if (i > 1)
        S(i) = S(i-1)*Ai;
    else
        S(i) = Ai;
    end
    
    Bi = compute_B(y_refs(i), u_refs(i), dt, A_, gamma_);
    T(i,i) = Bi;
    if (i > 1)
        for j = 1:i-1
            T(i,j) = T(i-1,j)*Ai;
        end
    end
    
end




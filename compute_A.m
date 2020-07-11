function A = compute_A(y_ref,u_ref, dt, A_, a_)

if (y_ref <= 0)
    y_ref = 0.001;
end

A = -dt*0.5*a_/A_*sqrt(2*9.82/(y_ref)) + 1;


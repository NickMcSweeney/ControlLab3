function B = compute_B(y_ref,u_ref, dt, A_, gamma_)

B = dt*gamma_ / A_;

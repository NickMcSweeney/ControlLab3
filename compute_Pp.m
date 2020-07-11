function [P p] = computePp(S,T,R,Q,y0)

P = T'*Q*T+R;
p = 2*T'*Q*S*y0;
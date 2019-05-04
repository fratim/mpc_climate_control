%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MPC cost calculation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INPUT:
%   x0: Initial state, dimension (3,1)
%   U:  Set of n subsequent inputs (n,2). 
%   P:  Terminal cost matrix. 
%   params:  Parameters (assumed to contain A,B,Q,R). 
% OUTPUT:
%   J:  Absolute cost. 
function J = compute_mpc_cost(x0,U,P,params)
N = size(U,2) + 1; 
A = params.A; B = params.B; Q = params.Q; R = params.R; 
J = 0; 
x = x0;
for k = 1:N-1
    J = J + x'*Q*x + U(:,k)'*R*U(:,k); 
    x = A*x + B*U(:,k); 
end
J = J + x'*P*x; 
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (Infinite Horizon) LQR controller
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INPUT:
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Cooling power, dimension (2,1)
function p = controller_lqr(T)
% controller variables
persistent param;
% initialize controller, if not done already
if isempty(param)
    param = init();
end
% normalize state. 
x = T - param.T_sp; 
% compute control action.
u = -param.F_inf*x;
% denormalize control input. 
p = u + param.p_sp; 
end

function param = init()
param = compute_controller_base_parameters;
A = param.A; 
B = param.B; 
R = param.R; 
Q = param.Q; 
[param.F_inf,param.P_inf,~] = dlqr(A,B,Q,R); 
end

% The infinite horizon cost under the LQR control can be obtained 
% by finding the infinite horizon terminal weight P_inf, such that
% J_LQR^inf(x(0)) = x(0)^T P_inf x(0). P_inf can be calculated by 
% solving the Algebraic Riccatti equation. 
function J_inf = cost(param, x)
assert ~is_empty(param)
J_inf = x.'*param.P_inf*x; 
end
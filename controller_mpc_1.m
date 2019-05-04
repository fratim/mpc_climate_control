%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MPC 1 controller
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INPUT:
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Cooling power, dimension (2,1)
function p = controller_mpc_1(T)
% controller variables
persistent param yalmip_optimizer
% initialize controller, if not done already
if isempty(param)
    [param, yalmip_optimizer] = init();
end
% normalize state. 
x = T - param.T_sp; 
% compute control action.
[u_mpc,errorcode] = yalmip_optimizer(x); 
if (errorcode ~= 0)
      warning('MPC infeasible');
end
% denormalize control input. 
p = u_mpc + param.p_sp;
end

function [param, yalmip_optimizer] = init()
% initializes the controller on first call and returns parameters and
% Yalmip optimizer object
param = compute_controller_base_parameters;
N = 30;
A = param.A; 
B = param.B; 
Q = param.Q; 
R = param.R; 
nx = size(A,1);
nu = size(B,2);
Ucons = param.Ucons; 
Xcons = param.Xcons; 
% initialize objective function and constraints (state constraint in the 
% k=0 step is not enforced due to numerical reasons). 
U = sdpvar(repmat(nu,1,N-1),repmat(1,1,N-1),'full'); %#ok<REPMAT>
X = sdpvar(repmat(nx,1,N),repmat(1,1,N),'full'); %#ok<REPMAT>
objective = 0;
constraints = [];
for k = 1:N-1  
    X{k+1} = A*X{k} + B*U{k};  
    constraints = [constraints; 
                   Ucons(:,1)<=U{k}<=Ucons(:,2); 
                   Xcons(:,1)<=X{k}<=Xcons(:,2)];  %#ok<AGROW>
    objective = objective + X{k}'*Q*X{k} + U{k}'*R*U{k};
end
objective = objective + X{N}'*Q*X{N};
% initialize (yalmip) mpc problem. 
ops = sdpsettings('verbose',2,'solver','quadprog');
yalmip_optimizer = optimizer(constraints,objective,ops,[],U);
end
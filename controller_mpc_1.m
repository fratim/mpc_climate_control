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

% [u_mpc,errorcode] = yalmip_optimizer(...);
% if (errorcode ~= 0)
%       warning('MPC infeasible');
% end
% p = ...;
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
U = sdpvar(repmat(nu,1,N-1),repmat(1,1,N-1),'full');
X = sdpvar(repmat(nx,1,N),repmat(1,1,N),'full');
objective = 0;
constraints = [Ucons(1,1)<=U{1}(1)<=Ucons(1,2); 
               Ucons(2,1)<=U{2}(1)<=Ucons(2,2); 
               X{];
for k = 1:N-1
  constraints = [constraints; 
                 Ucons(1,1)<=U{1}(k+1)<=Ucons(1,2); 
                 Ucons(2,1)<=U{2}(k+1)<=Ucons(2,2); 
                 Xcons(1,1)<=X{1}(k+1)<=Xcons(1,2); 
                 Xcons(2,1)<=X{2}(k+1)<=Xcons(2,2); 
                 Xcons(3,1)<=X{3}(k+1)<=Xcons(3,2)];  %#ok<AGROW>
  objective = objective + ;
end
% objective = objective + ... ;
% % initialize (yalmip) mpc problem. 
% ops = sdpsettings('verbose',0,'solver','quadprog');
% fprintf('JMPC_dummy = %f',value(objective));
% yalmip_optimizer = optimizer(constraints,objective,ops,... , ... );
end
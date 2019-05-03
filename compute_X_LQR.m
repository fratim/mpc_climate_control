%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Computation of explicit invariant set
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% OUTPUT:
%   A_x, b_x: Describes polytopic X_LQR = {x| A_x * x <= b_x}
function [A_x, b_x] = compute_X_LQR
    % get basic controller parameters
    param = compute_controller_base_parameters;
    % compute invariant set using MPT toolbox. 
    system = LTISystem('A', param.A, 'B', param.B);
    system.x.min = param.Xcons(:,1);
    system.x.max = param.Xcons(:,2);
    system.u.min = param.Ucons(:,1);
    system.u.max = param.Ucons(:,2);
    inv_set = system.invariantSet(); 
    A_x = inv_set.A; 
    b_x = inv_set.b; 
end


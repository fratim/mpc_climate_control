%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation - Main. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialization. 
clear all  %#ok<CLALL>
close all
addpath(genpath(cd));
load('system/parameters_scenarios.mat');

% Load system. 
system_params = compute_controller_base_parameters;

%% LQR simulation - Scenario I. 
clear controller_lqr; 
T0 = system_params.T_sp + [3;1;0];  
[T, p] = simulate_truck(T0, @controller_lqr,scen1);

%% LQR simulation - Scenario II. 
clear controller_lqr; 
T0 = system_params.T_sp + [-1.0;-0.1;-4.5]; 
[T, p] = simulate_truck(T0, @controller_lqr,scen2);

%% LQR - Invariant set. 
[A_x, b_x] = compute_X_LQR(); 

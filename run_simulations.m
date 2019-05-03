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

%% LQR simulation. 
% clear persisten variables of function controller_lqr
clear controller_lqr; 
% set initial temperature. 
x0 = [3;1;0]; 
T0 = system_params.T_sp + x0; 
% execute simulation starting from T0_1 using LQR controller. 
[T, p] = simulate_truck(T0, @controller_lqr,scen1);
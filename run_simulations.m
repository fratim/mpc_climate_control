%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation - Main. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialization. 
clc; clear all; close all; %#ok<CLALL>
addpath(genpath(cd));
load('system/parameters_scenarios.mat');

% Load system. 
system_params = compute_controller_base_parameters;

% Initial conditions. 
T0_1 = system_params.T_sp + [3;1;0]; 
T0_2 = system_params.T_sp + [-1.0;-0.1;-4.5]; 

%% LQR simulation.
clear controller_lqr; 
[T, p] = simulate_truck(T0_1, @controller_lqr,scen1);
print('outs/lqr_scen1_to1','-dpng')

clear controller_lqr; 
[T, p] = simulate_truck(T0_2, @controller_lqr,scen2);
print('outs/lqr_scen2_to2','-dpng')

%% LQR - Invariant set. 
[A_x, b_x] = compute_X_LQR(); 

%% MPC1 simulation - Scenario I. 
clear yalmip_optimizer; 
[T, p] = simulate_truck(T0_1, @controller_mpc_1,scen1);
print('outs/mpc_1_scen1_to1','-dpng')

clear yalmip_optimizer; 
[T, p] = simulate_truck(T0_2, @controller_mpc_1,scen1);
print('outs/mpc_1_scen1_to2','-dpng')

%% MPC2 simulation - Scenario I. 
clear yalmip_optimizer; 
[T, p] = simulate_truck(T0_1, @controller_mpc_2,scen1);
print('outs/mpc_2_scen1_to1','-dpng')

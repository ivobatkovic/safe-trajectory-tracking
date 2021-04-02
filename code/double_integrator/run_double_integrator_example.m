clear;
close all;

% Add subfolders and make directory for data
addpath(genpath('.'));
if not(isfolder(gen_path({'data','double_integrator'})))
  mkdir(gen_path({'data','double_integrator'}));
end

% Initial simulation point
xInit = [0;0;0];

% Simulation time
sim_time = 18;

% Save file location
save_file = gen_path({'data','double_integrator','double_integrator_'});

%% Simulate MPC
fprintf('Simulating double integrator MPC \n');
double_integrator_mpc(xInit,[save_file 'mpc.mat'],sim_time);
fprintf('------------------------------------------\n');
%% Simulate MPFTC
fprintf('Simulating double integrator MPFTC \n');
double_integrator_mpftc(xInit,[save_file 'mpftc.mat'],sim_time);
fprintf('------------------------------------------\n');
%% Simulate safe MPFTC
fprintf('Simulating double integrator safe MPFTC\n');
double_integrator_safe_mpftc(xInit,[save_file 'safe_mpftc.mat'],sim_time);
fprintf('------------------------------------------\n');

%% Plot results
plot_double_integrator_results;

%% Report avg runtime for MPFC at point 1
mpc = load(gen_path(...
            {'data','double_integrator','double_integrator_mpc.mat'}));
fprintf('Average runtime for MPC %.3f[s]\n',mean(mpc.time_array));
mpftc = load(gen_path(...
            {'data','double_integrator','double_integrator_mpftc.mat'}));
fprintf('Average runtime for MPFC %.3f[s]\n',mean(mpftc.time_array));
safe_mpftc = load(gen_path(...
        {'data','double_integrator','double_integrator_safe_mpftc.mat'}));
fprintf('Average runtime for MPFC %.3f[s]\n',mean(safe_mpftc.time_array));
fprintf('\n\n');

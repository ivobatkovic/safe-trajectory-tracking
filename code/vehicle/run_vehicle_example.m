clear;
close all;

% Add subfolders and make directory for data
addpath(genpath('.'));
if not(isfolder(gen_path({'data','vehicle'})))
  mkdir(gen_path({'data','vehicle'}));
end

% Initial simulation points
xInit_1 = [-30; -1; pi/8; 0];
xInit_2 = [-17.5; -1; pi/4; 0];
xInit_3 = [-10; 3; pi/4; 0];

% Simulation times
sim_time_1 = 10;
sim_time_2 = 7.5;
sim_time_3 = 5;

% File names
mpftc_file = gen_path({'data','vehicle','vehicle_mpftc_'});
mpfc_file = gen_path({'data','vehicle','vehicle_mpfc_'});

%% Simulate MPFC
fprintf('Simulating MPFC for three initial conditions\n');
vehicle_mpfc(xInit_1,[mpfc_file '1.mat'],sim_time_1);
vehicle_mpfc(xInit_2,[mpfc_file '2.mat'],sim_time_2);
vehicle_mpfc(xInit_3,[mpfc_file '3.mat'],sim_time_3);
fprintf('------------------------------------------\n');
%%  Simulate MPFTC for point 1
fprintf('Simulating MPFTC with w = 1e-4\n');
vehicle_mpftc(xInit_1,[mpftc_file '1_small.mat'],sim_time_1,1e-4);
vehicle_mpftc(xInit_2,[mpftc_file '2_small.mat'],sim_time_2,1e-4);
vehicle_mpftc(xInit_3,[mpftc_file '3_small.mat'],sim_time_3,1e-4);
fprintf('------------------------------------------\n');
%% Simulate MPFTC for point 2
fprintf('Simulating MPFTC with w = 1e1\n');
vehicle_mpftc(xInit_1,[mpftc_file '1_mid.mat'],sim_time_1, 1e1);
vehicle_mpftc(xInit_2,[mpftc_file '2_mid.mat'],sim_time_2, 1e1);
vehicle_mpftc(xInit_3,[mpftc_file '3_mid.mat'],sim_time_3, 1e1);
fprintf('------------------------------------------\n');
%% Simulate MPFTC for point 3
fprintf('Simulating MPFTC with w = 1e4\n');
vehicle_mpftc(xInit_1,[mpftc_file '1_large.mat'],sim_time_1,1e4);
vehicle_mpftc(xInit_2,[mpftc_file '2_large.mat'],sim_time_2,1e4);
vehicle_mpftc(xInit_3,[mpftc_file '3_large.mat'],sim_time_3,1e4);
fprintf('------------------------------------------\n');

%% Plot results
plot_vehicle_results;

%% Report avg runtime for MPFC at point 1
mpfc = load(gen_path({'data','vehicle','vehicle_mpfc_1.mat'}));
fprintf('Average runtime for MPFC %.3f[s]\n',mean(mpfc.time_array));

mpftc = load(gen_path({'data','vehicle','vehicle_mpftc_1_mid.mat'}));
fprintf('Average runtime for MPFTC %.3f[s]\n',mean(mpftc.time_array));

%%
fprintf('\n\n');
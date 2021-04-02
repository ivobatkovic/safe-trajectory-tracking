clear;
close all;

% Add subfolders and make directory for data
addpath(genpath('.'));
if not(isfolder(gen_path({'data','robotic_arm'})))
  mkdir(gen_path({'data','robotic_arm'}));
end

% Initial condition
xInit = [-5.86; 2.43; 0; 0; 0];
sim_time = 25;

% Save file location
save_file = gen_path({'data','robotic_arm','robotic_arm_mpftc.mat'});

%% Simulate MPC
fprintf('Simulating robotic arm MPFTC \n');
robotic_arm(xInit,save_file,sim_time);
fprintf('------------------------------------------\n');

%% Plot results
plot_robotic_arm_results;

%% Report avg runtime for MPFTC
data_file = gen_path({'data','robotic_arm','robotic_arm_mpftc.mat'});
mpftc = load(data_file);
fprintf('Average runtime for robotic arm MPFTC %.3f[s]\n',...
        mean(mpftc.time_array));
      
%%
fprintf('\n\n');
clear;
close all;

fprintf('Generating plots for vehicle example:\n');
% Change to true if tightfig.m and tight_subplot.m from 
% Matlab File Exchange are present
third_party_plot = false;
if exist('tightfig.m','file') && exist('tight_subplot.m','file')
  third_party_plot = true;
end

% Define colors
gray = 0.5*[1  1 1];
black = [0 0 0];
orange  = [0.85 0.325 0.09];

% Load data
xs0 = load(gen_path({'data','vehicle','vehicle_mpftc_1_small.mat'}));
xs1 = load(gen_path({'data','vehicle','vehicle_mpftc_2_small.mat'}));
xs2 = load(gen_path({'data','vehicle','vehicle_mpftc_3_small.mat'}));

x0 = load(gen_path({'data','vehicle','vehicle_mpftc_1_mid.mat'}));
x1 = load(gen_path({'data','vehicle','vehicle_mpftc_2_mid.mat'}));
x2 = load(gen_path({'data','vehicle','vehicle_mpftc_3_mid.mat'}));

xl0 = load(gen_path({'data','vehicle','vehicle_mpftc_1_large.mat'}));
xl1 = load(gen_path({'data','vehicle','vehicle_mpftc_2_large.mat'}));
xl2 = load(gen_path({'data','vehicle','vehicle_mpftc_3_large.mat'}));


xf0 = load(gen_path({'data','vehicle','vehicle_mpfc_1.mat'}));
xf1 = load(gen_path({'data','vehicle','vehicle_mpfc_2.mat'}));
xf2 = load(gen_path({'data','vehicle','vehicle_mpfc_3.mat'}));

ref = xs0;

%% Plot closed loop trajectories of states
fprintf('Figure 1...');
tx = ref.tx;
tu = ref.tu;
k = 1;
nPlots = 6;
data{k}.nPlots = nPlots;
data{k}.x = {tx;tx;...
             tx;tx;...
             tx;tx}; 
data{k}.y = {x0.ref(1,:)/6;x0.X(1,:)/6;...
             x0.ref(2,:);x0.X(2,:);...
             x0.ref(3,:);x0.X(3,:)}; 
data{k}.color = {gray;black;
                 gray;black;
                 gray;black};
data{k}.linestyle = {'-';'-';
                     '--';'--';...
                     '-.';'-.'};
data{k}.linewidth = {6;3;
                     6;3;
                     6;3};
data{k}.title = '';
data{k}.xlabel = '$t$ [s]'; data{k}.ylabel = '$x$[m], $y$[m], $\psi$[rad]';
data{k}.grid = 1;
data{k}.fontsize = 22;
data{k}.gca_fontsize = 22;
data{k}.legend_loc = 'northeast';
data{k}.legend = {'','$x/6$',...
                  '','$y$',...
                  '','$\psi$',...
                   '','','',...
                   '','',''};
                 
k = k+1;
nPlots = 4;
data{k}.nPlots = nPlots;
data{k}.x = {tu;tu;...
             tu;tu;}; 
data{k}.y = {x0.refu(1,1:end-1)/6;x0.U(1,:)/6;...
             x0.refu(2,1:end-1);x0.U(2,:)}; 
data{k}.color = {gray;black;
                 gray;black;
                 gray;black};
data{k}.linestyle = {'-';'-';
                     '--';'--';...
                     ':';'-'};
data{k}.linewidth = {6;3;
                     6;3;
                     6;3};
data{k}.title = '';
data{k}.xlabel = '$t$ [s]'; data{k}.ylabel = '$u_1$ [m/s], $u_2$ [rad]';
data{k}.grid = 1;
data{k}.fontsize = 23;
data{k}.gca_fontsize = 22;
data{k}.legend_loc = 'northeast';
data{k}.legend = ...
  {'','$u_1/6$',...
  '','$u_2$',...
  '$\mathrm{atan}(\frac{\partial{}p_2}{\partial\theta})$','$\psi$',...
  '','','',...
  '','',''};
                 
opts.third_party = third_party_plot;
opts.dim = [1 2]; opts.gap = [0.15 0.06]; 
opts.margh = [0.1 0.1]; opts.margw = [0.1 0.1];
px = get(0,'screensize');
gh=figure; set(gh,'position',[20 px(4)/3 px(3) px(3)/4]);
ax = plot_trajectories(data,opts); 
grid on

% Save figure
save_file = gen_path({'data','vehicle','vehicle_states.eps'}); 
saveas(gcf,save_file,'epsc');

dst = gen_path({'..','writing','figures','vehicle_states.eps'});
copyfile(save_file, dst, 'f');
fprintf('done\n');
%% Plot closed loop trajectories in the x-y plane
fprintf('Figure 2...');
k = 1;
nPlots = 13;
data{k}.nPlots = nPlots;
data{k}.x = {ref.ref(1,:);...
             xs0.X(1,:);x0.X(1,:);xl0.X(1,:);xf0.X(1,:);...
             xs1.X(1,:);x1.X(1,:);xl1.X(1,:);xf1.X(1,:);...
             xs2.X(1,:);x2.X(1,:);xl2.X(1,:);xf2.X(1,:)}; 
data{k}.y = {ref.ref(2,:);...
             xs0.X(2,:);x0.X(2,:);xl0.X(2,:);xf0.X(2,:);...
             xs1.X(2,:);x1.X(2,:);xl1.X(2,:);xf1.X(2,:);...
             xs2.X(2,:);x2.X(2,:);xl2.X(2,:);xf2.X(2,:)}; 
data{k}.color = {gray;...
                 black; black; black; orange;...
                 black; black; black; orange;
                 black; black; black; orange};
data{k}.linestyle = {'-';...
                     '--';'-.';':';'--';...
                     '--';'-.';':';'--';...
                     '--';'-.';':';'--';};
data{k}.linewidth = {6;...
                     3; 3; 3; 3;...
                     3; 3; 3; 3;...
                     3; 3; 3; 3};
data{k}.title = '';
data{k}.xlabel = '$x$ [m]'; data{k}.ylabel = '$y$ [m]';
data{k}.fontsize = 22;
data{k}.gca_fontsize = 22;
data{k}.grid = 'on';
data{k}.legend = {'',...
                  'MPFTC, $w=10^{-4}$','MPFTC, $w=10$',...
                  'MPFTC, $w=10^4$','MPFC'...
                   '','','','',...
                   '','','',''};
data{k}.legend_loc = 'northwest';

opts.third_party = third_party_plot;
opts.dim = [1 1]; opts.gap = [0.15 0.04]; 
opts.margh = [0.1 0.1]; opts.margw = [0.1 0.1];

px = get(0,'screensize');
gh=figure; set(gh,'position',[20 px(4)/3 px(3)/2 px(3)/4]);
plot_trajectories(data,opts); 
ylim([-2.1 5]);
xlim([-30 0]);

x0p = x0.X(1:2,1)+[4;3];
th0 = text(x0p(1)-2.2,x0p(2)-.45,'\boldmath$(-30,-1,\frac{\pi}{8})$');
t0_1 = annotation('textarrow',[0.23 0.12],[0.525  0.36],'String','',...
  'interpreter','latex'); t0_1.FontSize=18; t0_1.LineWidth=2;

x1p = x1.X(1:3,1);
th1 = text(x1p(1)-2.2,x1p(2)-.45,'\boldmath$(-17.5,-1,\frac{\pi}{4})$');

x2p = x2.X(1:3,1);
th2 = text(x2p(1)-4.5,x2p(2)-.35,'\boldmath$(-10,3,\frac{\pi}{4})$');

set([th0 th1 th2],'interpreter','latex','fontsize',24,'fontweight','bold')

% Save figure;
save_file = gen_path({'data','vehicle','vehicle_closed.eps'}); 
saveas(gcf,save_file,'epsc');

dst = gen_path({'..','writing','figures','vehicle_closed.eps'});
copyfile(save_file, dst, 'f');
fprintf('done\n');

%%
fprintf('Done generating plots for vehicle example\n');
fprintf('-----------------------------------------\n');
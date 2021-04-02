clear;
close all;

fprintf('Generating plots for double integrator example:\n');

% Change to true if tightfig.m and tight_subplot.m from 
% Matlab File Exchange are present
third_party_plot = false;
if exist('tightfig.m','file') && exist('tight_subplot.m','file')
  third_party_plot = true;
end

% Load data
mpc = load(gen_path(...
         {'data','double_integrator','double_integrator_mpc.mat'})); 
safe_mpftc = load(gen_path(...
         {'data','double_integrator','double_integrator_safe_mpftc.mat'})); 
mpftc = load(gen_path(...
         {'data','double_integrator','double_integrator_mpftc.mat'})); 

% Define colors
co = get(gca,'colororder'); close;
gray = 0.5*[1  1 1];
black = [0 0 0]; 
blue = co(3,:);
red = [.9 .05 .01];
orange = co(1,:);
yellow = co(3,:);
alpha = 1;

%% Plot closed loop states
fprintf('Figure 3...');
% Plot closed loop trajectories
k = 1;
nPlots = 6;
data{k}.nPlots = nPlots;
data{k}.x = {mpc.tx;mpftc.tx(15:end);safe_mpftc.tx(10:end);...
             mpc.tx; mpftc.tx; safe_mpftc.tx}; 
data{k}.y = {mpc.ref(1,:);mpftc.ref(1,15:end);safe_mpftc.ref(1,10:end);...
             mpc.X(1,:); mpftc.X(1,:); safe_mpftc.X(1,:);};
data{k}.color = {[gray alpha];[gray alpha];[gray alpha];...
                  black;blue;orange;[orange alpha];red;red;red};
data{k}.linestyle = {'-';'--';'-';...
                     '-';'--';':';'-';':';':';':'};
data{k}.linewidth = {3; 3;3;3;3;3};
data{k}.title = '';
data{k}.xlabel = '$t$ [s]'; data{k}.ylabel = '$p$ [m]';
data{k}.fontsize = 23;
data{k}.gca_fontsize = 22;
data{k}.grid ='on';
data{k}.legend = {''; '';'';'MPC'; 'MPFTC'; 'Safe MPFTC'; ''; };

k = k+1;
nPlots = 5;
data{k}.nPlots = nPlots;
data{k}.x = {mpc.tx; mpc.tx; mpftc.tx; safe_mpftc.tx;safe_mpftc.tx}; 
data{k}.y = {safe_mpftc.X(2,:)*0;safe_mpftc.ref(2,:);...
             mpc.X(2,:); mpftc.X(2,:); safe_mpftc.X(2,:)};
data{k}.color = {red; gray; black; blue; orange};
data{k}.linestyle = {'-';'-';'-';'--';':';'-';'-';'-'};
data{k}.linewidth = {3; 3; 3; 3; 3; 3; 3; 3};
data{k}.title = '';
data{k}.xlabel = '$t$ [s]'; data{k}.ylabel = '$\dot{p}$ [m/s]';
data{k}.fontsize = 23;
data{k}.gca_fontsize = 22;
data{k}.grid ='on';
data{k}.legend_loc = 'north';
data{k}.legend = {'';'';'MPC';'MPFTC';'Safe MPFTC';'';'';''};

k = k+1;
nPlots = 5;
data{k}.nPlots = nPlots;
data{k}.x = {mpc.tu; mpc.tu; mpc.tu; mpftc.tu; safe_mpftc.tu}; 
data{k}.y = {-1+0*safe_mpftc.U(1,:);5+0*safe_mpftc.U(1,:);...
             mpc.U(1,:); mpftc.U(1,:); safe_mpftc.U(1,:)};
data{k}.color = {red; red; black; blue; orange;};
data{k}.linestyle = {'-'; '-'; '-'; '--'; ':'; ':'; ':'; ':'; ':'};
data{k}.linewidth = {3; 3; 3; 3; 3};
data{k}.title = '';
data{k}.xlabel = '$t$ [s]'; data{k}.ylabel = '$a$ [m/s$^2$]';
data{k}.fontsize = 23;
data{k}.gca_fontsize = 22;
data{k}.grid ='on';
data{k}.legend_loc = 'north';
data{k}.legend = {'','','MPC','MPFTC','Safe MPFTC'};

opts.third_party = third_party_plot;
opts.dim = [1 3]; opts.gap = [0.15 0.045]; 
opts.margh = [0.1 0.1]; opts.margw = [0.1 0.1];
px = get(0,'screensize');
gh=figure; set(gh,'position',[20 px(4)/3 px(3) px(3)/3.5]);
axe = plot_trajectories(data,opts);

% Fix axes for first plot
axes(axe(1)); ylim([0 35]); xlim([0 18]);
h = fill([0 15 15 0],[20 20 50 50],red);
chH = get(gca,'Children');
set(gca,'Children',[chH(2:end);chH(1)])
set(h,'edgecolor',red,'facealpha',.7);
set(axe(1).Legend,'location','northwest')
set(h,'displayname','Obstacle')

% Fix axes for second plot
axes(axe(2)); ylim([-.5 6]); xlim([0 18]);
axes(axe(3)); ylim([-1.5 5.5]);
set(axe(2).Legend,'location','north');

% Fix axes for third plot
axes(axe(3)); xlim([0 18]);

% Plot open loop trajectories
x = safe_mpftc;
t = 0 : x.ts : size(x.XP_open,2)*x.ts - x.ts;
orange_a = [orange 0.05];
set(gcf,'currentaxes',axe(1))
for k = 1 : size(mpftc.U,2)
  plot(t+x.ts*(k-1),x.XP_open(k,:),...
    'color',orange_a,'HandleVisibility','off','linewidth',3)
end

set(gcf,'currentaxes',axe(2))
for k = 1 : size(mpftc.U,2)
  plot(t+x.ts*(k-1),x.XV_open(k,:),...
    'color',orange_a,'HandleVisibility','off','linewidth',3)
end

% Save figure;
save_file = gen_path(...
   {'data','double_integrator','double_integrator_states.eps'}); 
saveas(gcf,save_file,'epsc');

dst = gen_path(...
     {'..','writing','figures','double_integrator_states.eps'});
copyfile(save_file, dst, 'f');

fprintf('done\n');

%% Plot terminal safe set
fprintf('Figure 4...');

ts = safe_mpftc.ts;
K = safe_mpftc.K;

% Describe double integrator
Ac = [0 1; 0 0]; Bc = [0; 1];
sysc = ss(Ac,Bc,[],[]);
sysd = c2d(sysc,ts,'zoh');
Ad = sysd.A; Bd = sysd.B;

% Reference
r = @(t) [4*t; 4];

% Initial safe set  ( -500 <= x <= 20; 0 <= v <= 0)
H = [0 1; 0 -1; 1 0; -1 0]; b = [0; 0;20; 500];
P0 = Polyhedron(H,b);

% Propagate safe set backwards to see all states that can come to zero in
% 50 time steps and x < 20

% -1 <= u <= 0 .. only interested in breaking scenarios
Hu = [1; -1]; bu = [0; 1]; 
for k =  1 : 50
  H = P0.H(:,1:2); b = P0.H(:,3);
  H = [H*Ad H*Bd; zeros(2,2) Hu]; b = [b; bu];
  P0 = projection(Polyhedron(H,b),[1 2]);
  P0.minHRep;
end

% Store polyhedras
P_hist = []; t_hist = [];
times = [0 2 3 4 5];
for k = times
  
  % Tau at terminal set
  t = k;
  
  % Terminal control + constrain velocity [0,5]
  H = [-K;K;0 1; 0 -1]; b = [5 - K*r(t); 1 + K*r(t); 5;0];
  P = intersect(Polyhedron(H,b),P0);
  
  % Terminal set in x-r^x space
  p = Polyhedron((P.V'-r(t))');
 
  P_hist = [P_hist p];
  t_hist = [t_hist (k-1)*ts];
  
end

% Plotting 
px = get(0,'screensize');
gh=figure; set(gh,'position',[20 px(4)/2 px(3)/2 280*1.2]);

% Use third-party 
if third_party_plot
  opts.third_party = third_party_plot;
  dim = [1 1]; gap = [0.0 0.00]; 
  margh = [0.1 -0.1 ]; margw = [0.12 0.02];
  axe = tight_subplot(dim(1),dim(2),gap, margh, margw);
  set(gcf,'currentaxes',axe);
end
co = get(gca,'colororder');
alpha = 0.7;

hold on;
p_h = [];
for k = 1: length(times)
  p = P_hist(k);
  p_h = [p_h p.plot('color',co(k,:),'alpha', alpha,'linewidth',1.5)];
end

l = legend(p_h,'$\mathcal{X}_\mathbf{r}^\mathrm{f}(0)$',...
              '$\mathcal{X}_\mathbf{r}^\mathrm{f}(2)$',...
              '$\mathcal{X}_\mathbf{r}^\mathrm{f}(3)$',...
              '$\mathcal{X}_\mathbf{r}^\mathrm{f}(4)$',...
              '$\mathcal{X}_\mathbf{r}^\mathrm{f}(5)$');
set(l,'interpreter','latex','fontsize',24,...
  'NumColumns',5,'location','northoutside')
daspect([5 1 1])
axis tight
title('');
xh = xlabel('$p-tv^\mathrm{r}$ [m]');
yh = ylabel('$\dot{p}-v^\mathrm{r}$ [m/s]');
set([xh yh],'fontsize',25,'interpreter','latex');
set(gca,'fontsize',25);
grid on; box on
set(gcf,'renderer','Painters')

if third_party_plot
  tightfig;
end

% Save figure;
save_file = gen_path(...
   {'data','double_integrator','double_integrator_safe_terminal_set.eps'}); 
saveas(gcf,save_file,'epsc');

dst = gen_path(...
     {'..','writing','figures','double_integrator_safe_terminal_set.eps'});
copyfile(save_file, dst, 'f');

fprintf('done\n');

%%
fprintf('Done generating plots for double integrator example\n');
fprintf('---------------------------------------------------\n');
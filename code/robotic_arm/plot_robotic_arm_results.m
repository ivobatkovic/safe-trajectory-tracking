clear;
close all;

fprintf('Generating plots for robotic arm example:\n');

% Change to true if tightfig.m and tight_subplot.m from 
% Matlab File Exchange are present
third_party_plot = false;
if exist('tightfig.m','file') && exist('tight_subplot.m','file')
  third_party_plot = true;
end

% Define colors
co = get(gca,'colororder'); close all
yellow = co(3,:);
gray = 0.5*[1  1 1];
black = [0 0 0];
red = [1 0 0];

% Load data
load(gen_path({...
      'data','robotic_arm','robotic_arm_mpftc.mat'}));
reference = ...
  load(gen_path({...
      'data','robotic_arm','robotic_arm_reference.mat'}));
%% Plot closed loop trajectories
fprintf('Figure 5...');
% States q1, q2
k = 1;
nPlots = 4;
data{k}.nPlots = nPlots;
data{k}.x = {tx;tx;tx;tx}; 
data{k}.y = {ref(1,:);X(1,:);ref(2,:);X(2,:)};
data{k}.color = {gray; black; gray; black};
data{k}.linestyle = {'-';'-';'--';'--';};
data{k}.linewidth = {5; 3 ; 5; 3; 2};
data{k}.title = 'Joint Angles';
data{k}.xlabel = '$t$ [s]'; data{k}.ylabel = '$q_1$, $q_2$ [rad]';
data{k}.fontsize = 23;
data{k}.gca_fontsize = 22;
data{k}.grid = 'on';
data{k}.legend_loc = 'northeast';
data{k}.legend = {'','$q_1$','','$q_2$','e','f','g'};

% States dot q1, dot q2
k = k+ 1;
nPlots = 4;
data{k}.nPlots = nPlots; aa = linspace(0,2*pi,100);
data{k}.x = {tx;tx;tx; tx}; 
data{k}.y = {ref(3,:);ref(4,:);X(3,:);X(4,:)};
data{k}.color = {gray; gray;black;black};
data{k}.linestyle = {'-';'--';'-';'--'};
data{k}.linewidth = {5;5;3;3};
data{k}.title = 'Joint Velocities';
data{k}.xlabel = '$t$ [s]'; 
data{k}.ylabel = '$\dot{q}_1$, $\dot{q}_2$ [rad/s]';
data{k}.fontsize = 23;
data{k}.gca_fontsize = 22;
data{k}.legend_loc = 'northeast';
data{k}.grid = 'on';
data{k}.legend = {'','','$\dot{q}_1$','$\dot{q}_2$','e','f','g'};

% Controls u1, u2
k = k+ 1;
nPlots = 6;
data{k}.nPlots = nPlots;
data{k}.x = {tu;tu;tu;tu;tu;tu}; 
data{k}.y = {-4000+0*tu;4000+0*tu;refu(1,:);refu(2,:);U(1,:);U(2,:)};
data{k}.color = {red; red; gray;gray; black; black};
data{k}.linestyle = {'-';'-';'-';'--';'-';'--'};
data{k}.linewidth = {3;3;5;5;3;3};
data{k}.title = 'Torques';
data{k}.xlabel = '$t$ [s]'; data{k}.ylabel = '$u_1$, $u_2$ [Nm]';
data{k}.fontsize = 23;
data{k}.gca_fontsize = 22;
data{k}.grid = 'on';
data{k}.legend_loc = 'northeast';
data{k}.legend = {'','','','','$u_1$','$u_2$','e','f','g'};

% Constraint g(x,u)
k = k+ 1;
nPlots = 2;
data{k}.nPlots = nPlots;
data{k}.x = {tx;tx}; 
data{k}.y = {0*tx;0.1-((X(1,:)-X_obs(1,:)).^2+(X(2,:)-X_obs(2,:)).^2)};
data{k}.color = {red; black};
data{k}.linestyle = {'-';'--'};
data{k}.linewidth = {4; 3};
data{k}.title = 'Constraint $g(x_k,u_k)\leq{}0$';
data{k}.xlabel = '$t$ [s]'; data{k}.ylabel = '$g(x_{k},u_{k})$';
data{k}.fontsize = 23;
data{k}.gca_fontsize = 22;
data{k}.grid = 'on';
data{k}.legend_loc = 'northeast';
data{k}.legend = ...
        {'','','$\hat{r}_k-(x_{1,k}-x_k^{obs})^\top(x_{1,k}-x_k^{obs})$'};

opts.third_party = third_party_plot;
opts.dim = [2 2]; opts.gap = [0.2 0.06]; 
opts.margh = [0.1 0.1]; opts.margw = [0.1 0.1];
px = get(0,'screensize');
gh=figure; set(gh,'position',[20 px(4)/3 px(3) px(3)/3.1]);
ax = plot_trajectories(data,opts);

axes(ax(3)); ylim([-2500 2500]);
axes(ax(4)); ylim([-30 5]);

% Save figure
save_file = gen_path(...
          {'data','robotic_arm','robotic_arm_closed.eps'}); 
saveas(gcf,save_file,'epsc');

dst = gen_path(...
              {'..','writing','figures','robotic_arm_closed.eps'});
copyfile(save_file, dst, 'f');

fprintf('done\n');
%% Open loop trajectories here
fprintf('Figure 6...');
% Plot (q1,q2) for the closed loop simulation
k = 1;
nPlots = 2;
tIndex = 1;
nx = 5;
M = 25;
endTime = M+1;

padding = linspace(-5.3,0,1000);
data{k}.nPlots = nPlots; r = X_obs;
data{k}.x = {full(reference.p1(padding));...
             X_open(1+(tIndex-1)*nx,1:endTime);...
             X_obs_open(1+(tIndex-1)*3,1:endTime);tx}; 
data{k}.y = {full(reference.p2(padding));...
             X_open(2+(tIndex-1)*nx,1:endTime);...
             X_obs_open(2+(tIndex-1)*3,1:endTime);X(2,:)};
data{k}.color = {[gray 0.75]; black; red; black};
data{k}.linestyle = {'-';'-';'-';':';};
data{k}.linewidth = {5; 4 ; 4; 3; 2};
data{k}.title = '';
data{k}.xlabel = '$q_1$ [rad]'; data{k}.ylabel = '$q_2$ [rad]';
data{k}.fontsize = 18;
data{k}.gca_fontsize = 18;
data{k}.grid = 'on';
data{k}.legend_loc = 'northeast';
data{k}.legend = {'','$(q_1,q_2)$','$Obstacle$','$q_2$','e','f','g'};

opts.third_party = third_party_plot;
opts.dim = [1 1]; opts.gap = [0.15 0.04]; 
opts.margh = [0.1 0.1]; opts.margw = [0.1 0.1];
px = get(0,'screensize');
gh=figure; set(gh,'position',[400 px(4)/3 px(3)/3 px(3)/3]);
axe = plot_trajectories(data,opts);

axes(axe);
th = linspace(0,2*pi,100);
nskip = 2; opacity = .4;
for k = 1 : nskip : M+1
  x0 = X_obs_open(1+(tIndex-1)*3:3+(tIndex-1)*3,k);
  plot(x0(1)+sqrt(x0(3))*cos(th),...
       x0(2)+sqrt(x0(3))*sin(th),...
       'color',[red opacity],'handlevisibility','off','linewidth',2);
end
annotation('textarrow',[0.2 0.25],[0.55  0.7],...
  'String','$t=0$s','fontsize',18,'linewidth',2,...
  'units','normal','interpreter','latex');
annotation('textarrow',[0.2 0.22],[0.5  0.46],...
  'String','','fontsize',18,'linewidth',2);

tIndex = 130;
for k = 1 : nskip : M+1
  x0 = X_obs_open(1+(tIndex-1)*3:3+(tIndex-1)*3,k);
  plot(x0(1)+sqrt(x0(3))*cos(th),...
       x0(2)+sqrt(x0(3))*sin(th),...
       'color',[red opacity],'handlevisibility','off','linewidth',2);
end
plot(X_open(1+(tIndex-1)*nx,1:endTime),...
     X_open(2+(tIndex-1)*nx,1:endTime),...
     data{1}.linestyle{2},'linewidth',...
     data{1}.linewidth{2},'color',black,'handlevisibility','off');
annotation('textarrow',[0.4 0.38],[0.65  0.6],...
  'String','$t=3.9$s','fontsize',18,'linewidth',2,'interpreter','latex');
annotation('textarrow',[0.4 0.4],[0.65  0.56],'String','','linewidth',2);


tIndex = 400;
for k = 1 : nskip : M+1
  x0 = X_obs_open(1+(tIndex-1)*3:3+(tIndex-1)*3,k);
  plot(x0(1)+sqrt(x0(3))*cos(th),...
       x0(2)+sqrt(x0(3))*sin(th),...
       'color',[red opacity],'handlevisibility','off','linewidth',2);
end
plot(X_open(1+(tIndex-1)*nx,1:endTime),...
     X_open(2+(tIndex-1)*nx,1:endTime),...
    data{1}.linestyle{2},'linewidth',data{1}.linewidth{2},...
    'color',black,'handlevisibility','off');
plot(X_obs(1,1:tIndex),X_obs(2,1:tIndex),'color',red,...
    'linewidth',1,'handlevisibility','off');

annotation('textarrow',[0.7 0.65],[0.35  0.52],...
  'String','$t=12$s','interpreter','latex','fontsize',18,'linewidth',2);
t0_2 = annotation('textarrow',[0.7 0.665],[0.3  0.25],...
  'String','','linewidth',2);

% Save figure
save_file = gen_path(...
          {'data','robotic_arm','robotic_arm_open.eps'}); 
saveas(gcf,save_file,'epsc');

dst = gen_path(...
              {'..','writing','figures','robotic_arm_open.eps'});
copyfile(save_file, dst, 'f');
fprintf('done\n');

%%
fprintf('Done generating plots for robotic arm example\n');
fprintf('---------------------------------------------\n');
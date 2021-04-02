function axe = plot_trajectories(data,opts)

  axe = gcf;
  if opts.third_party
    axe = tight_subplot(opts.dim(1),opts.dim(2),...
                     opts.gap, opts.margh, opts.margw);
  end
  nPlots = opts.dim(1)*opts.dim(2);
  nDataEntries = size(data,2);
  for k = 1 : nPlots
    if k <= nDataEntries
      nData = data{k}.nPlots;
      
      if opts.third_party
        axes(axe(k));
      else
        subplot(opts.dim(1),opts.dim(2),k);
      end
      hold on; 
      th = title(data{k}.title); 
      xh = xlabel(data{k}.xlabel); yh = ylabel(data{k}.ylabel);
      set([th xh yh],'fontsize',data{k}.fontsize,'interpreter','latex');

      hLeg = []; legT = {};
      for kj = 1 : nData
        h = plot(data{k}.x{kj},data{k}.y{kj},data{k}.linestyle{kj},...
          'linewidth',data{k}.linewidth{kj},...
          'color',data{k}.color{kj});
        if ~isempty(data{k}.legend{kj})
          hLeg = [hLeg h];
          legT{end+1} = data{k}.legend{kj};
        end
      end
      
    end
    if ~isfield(data{k},'legend_loc')
      legend_loc = 'northwest';
    else
      legend_loc = data{k}.legend_loc;
    end
    if ~isempty(hLeg)
      legHandle = legend(hLeg,legT{:});
      set(legHandle,'fontsize',data{k}.fontsize,'interpreter','latex',...
        'location',legend_loc);
    end
    box on;
    set(gca,'fontsize',data{k}.gca_fontsize);
    set(0,'defaulttextInterpreter','latex');
    
    if isfield(data{k},'grid')
      grid on;
    end
  end
  if opts.third_party
    tightfig;
  end
  
  children = get(gcf,'children');
  if ~opts.third_party
    axe = children(end:-2:1);
  end
  
end


function plot_fminTest(check,F,pFvars,fname)

global state n_steps time_step burnIn

close all
vars = {'X','Y','Z','K','M','N'};
sname = {'u','v','w','p','q','r','xpos','ypos','zpos','\phi','\theta','\psi'};
% co = cbrewer('qual','Set3',12);
co = linspecer(12,'qualitative');
% co = distinguishable_colors(12);

setAxesDefaults
for k = 1:numel(pFvars)
    
    pFi = find(strcmp(vars,pFvars{k}));
    
    figure;
    set(groot,'defaultAxesColorOrder',co);
    set(gcf,'Units','normalized','Position',[0.05 0.075 0.9 0.8]);
    
    % Forces
    axF = axes;
    set(axF,'Position',[0.05 0.5 0.9 0.475]);
    plot(F.(pFvars{k}),'linewidth',3);
    
    % burn-in
    yl = get(axF,'YLim'); hold on;
    patch([1 1 burnIn burnIn],[yl(1) yl(2) yl(2) yl(1)],...
        'r','Edgecolor','none','FaceAlpha',0.05);
    
    if pFi<4
        ylabel(['Forces acting on ' pFvars{k}],'fontSize',18)
    else
        ylabel(['Moments acting on ' pFvars{k}],'fontSize',18)
    end
    % set(axF,'XTick',0:50:n_steps)
    xlim([0 n_steps])
    legend(F.names.(pFvars{k}))
    
    
    
    % Accelerations
    set(groot,'defaultAxesColorOrder','remove');
    axF1 = axes; c = pFi;
    set(axF1,'Position',[0.05 0.075 0.425 0.375]);
    if any(c == [3:6])
        p(1) = plot(rad2deg(state(:,c)),'linewidth',2); hold on;
        p(2) = plot(rad2deg(check(:,c)),'linewidth',2);
    else
        p(1) = plot(state(:,c),'linewidth',2); hold on;
        p(2) = plot(check(:,c),'linewidth',2);
    end
    
    % burn-in
    yl = get(axF1,'YLim'); 
    patch([1 1 burnIn burnIn],[yl(1) yl(2) yl(2) yl(1)],...
        'r','Edgecolor','none','FaceAlpha',0.05);
    
    xlim([0 n_steps])
    lb    = ylabel(sname{c});
    lb(2) = xlabel('Step');
    legend(['Observed (' num2str(1./time_step,2) ' Hz)'] ,...
        ['Simulated (' num2str(1./time_step,2) ' Hz)'])
    
    
    
    % Integrated
    axF2 = axes; c = pFi+6;
    set(axF2,'Position',[0.525 0.075 0.425 0.375]);
    if any(c == [10:12])
        p(1) = plot(rad2deg(state(:,c)),'linewidth',2); hold on;
        p(2) = plot(rad2deg(check(:,c)),'linewidth',2);
    else
        p(1) = plot(state(:,c),'linewidth',2); hold on;
        p(2) = plot(check(:,c),'linewidth',2);
    end
    
    % burn-in
    yl = get(axF2,'YLim'); 
    patch([1 1 burnIn burnIn],[yl(1) yl(2) yl(2) yl(1)],...
        'r','Edgecolor','none','FaceAlpha',0.05);
    
    xlim([0 n_steps])
    lb(3) = ylabel(sname{c});
    lb(4) = xlabel('Step');
    legend(['Observed (' num2str(1./time_step,2) ' Hz)'] ,...
        ['Simulated (' num2str(1./time_step,2) ' Hz)'])
    
    set(lb([1,3]),'fontSize',18,'fontWeight','bold')
    linkaxes([axF,axF1,axF2],'x')
    
    % Script source water-mark
    %------------------------------------------------------------------
    uicontrol('Style', 'text','String', [fname ' - ' datestr(clock)],...
        'Units','normalized',...
        'Position', [0 0 1 0.015]);
end
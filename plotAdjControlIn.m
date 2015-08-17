function plotAdjControlIn(w0)

global input

u_in = input(:,1) - mean(input(:,1));

% Correct elev offset:
%-------------------------------------------------------
u_in(1) = u_in(1) + w0(end-1);

% w0(end) = -w0(end); 

% Correct elev backlash:
%-------------------------------------------------------
u_in(u_in(:,1) < 0,1) = u_in(u_in(:,1) < 0,1) - w0(end);
u_in(u_in(:,1) > 0,1) = u_in(u_in(:,1) > 0,1) + w0(end);


figure;
set(gcf,'Units','normalized','Position',[0.1,0.3,0.8,0.5]); 
plot(rad2deg(input(:,1))); hold on;
plot(rad2deg(u_in)); axis tight
title(['Corrected control surfaces inputs: Elev offset =  '...
    sprintf('%.1f',rad2deg(w0(end-1))) ' Elev backlash = '...
    sprintf('%.1f',rad2deg(w0(end)))]) 
xlabel('Step')
ylabel('Degree')
axis 'auto y'

end
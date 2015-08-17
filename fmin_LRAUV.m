% fmin_LRAUV.m
% Error minimization program for LRAUV sim
% Last modified May 4, 2015 
% Ben Raanan

% EDIT THESE VARAIBLES TO SEARCH FOR DIFFERENT DATASETS
%--------------------------------------------------------------------------
vehicle = 'Daphne';                 % 'Tethys', 'Daphne', 'Makai'
year=2015;                          % 2010 - 2015
searchterm = '20150203T211256';     % see: help findmat_LRAUV

%--------------------------------------------------------------------------
clearvars -except vehicle year searchterm
clear global
close all

% Get work directory paths
%--------------------------------------------------------------------------
fname = which('depthPitch_LRAUV.m');
workd = '~/Documents/MATLAB/MBARI/';          % working directory
figd  = '~/Documents/MBARI/';                 % folder for saving figures
outmatfolder  = [workd 'mat/workver/'];
savematfolder = [workd 'sim/fminsim/mat/'];
%--------------------------------------------------------------------------
% save plot? (Y=1)
sv = 0;

% Declare globals
global n_steps time_step state input vehicle_coeffs optCoeff burnIn

% locate, fix and load dataset
%--------------------------------------------------------------------------
% get mat file path(s)
[matpath, matname] = findmat_LRAUV( vehicle, year, searchterm, outmatfolder );

% get parameters of intrest and fix time-series
[ filename ] = processLargeMAT_LRAUV( vehicle, year, searchterm, outmatfolder );
% full = load(matpath{:})

% load reduced .mat file (workable format)
[ time, time_step, xst, names, controls ] = initialize_LRAUV_SIM( filename{:} );
control = controls;

%
% Get mission names
%--------------------------------------------------------------------------
syslog  = getsyslog( matpath );    % requires connection to LRAUV server
mission = getmission( syslog, time, xst.z );


%% Elev offset
%--------------------------------------------------------------------------
missionName = {'sci2','science_to','circle_acoustic_contact','CANON_ESP_box'}; % mission.namelist
windowSize = 3600;
V = ElevDist_LRAUV(time, rad2deg(controls(:,1)'), xst.z,...
    xst.Cmd.verticalMode, mission, syslog, missionName, windowSize);

% Correct offset
ele_offset  = nanmean(V(4,:)); % deg
control(:,1) = controls(:,1) - deg2rad(ele_offset);
% control(:,2) = zeros(size(control(:,2)));

% Set up mass-shifting
%--------------------------------------------------------------------------
%{
mass        =   147.8671 + 5.14;         % kg Flooded Vehicle total mass (new batt += 5.14)
movableMass =   26 + 5.14;               % kg Battary movable mass (new batt = 31.14kg)
dropWtMass  =   1.0;              % kg Mass of the drop weight #1, kg
dropWtX     =  -0.1330;           % m  X location of the drop weight #1, m

xst.mass_p(168:end) = xst.mass_p(167);
Xmass = (movableMass.*xst.mass_p + dropWtMass*dropWtX)./mass;
% control(:,3) = zeros(size(Xmass'));
%}
%}



% INITIALIZE OPTIMIZATION
%--------------------------------------------------------------------------
timeIn  = datenum('04-Feb-2015 18:15:00'); % '01-May-2015 04:40:00'
timeEval    = 60*24; % sec, evaluation run time
timeIni = closest(timeIn,time);

% initiate first step and set runtime
%-----------------------------------------------------
startPoint = timeIni;
n_steps = fix(timeEval/time_step); %size(ui,1); 
n = startPoint:startPoint+n_steps;
burnIn = 175;

theta = xst.theta(n(1):n(end-1))';
% q = xst.q(n(1):n(end-1))';

% STATE AND INPUT VECTORS:
%-----------------------------------------------------
% state = [u v w p q r xpos ypos zpos phi theta psi]'
% input = [ delta_s delta_r Xprop Kprop]'
input = control(n,:);
state = zeros(length(n),12);

% u very noisy - for now replace u with commanded speed 
names{1} = 'uCmd';
xst.uCmd = xst.Cmd.speedCmd;

% Bin avg to reduce noise then interp
%-----------------------------------------------------
int = 13; % 5 sec bin avg
tbin = median(reshape(time, int, length(time)/int),1);
for c=[1:6,9,10:12];
    
    if any(c==1:4)
        tmp = xst.(names{c});
        bin = mean(reshape(tmp, int, length(time)/int),1);
        bi  = interp1(tbin,bin,time);
        state(:,c) = bi(n)';
    else
        state(:,c) = xst.(names{c})(n)';
    end
end;


%% INITIALIZE COEFFICIENT STRUCT:
%-----------------------------------------------------
rho = nanmean(SW_Density(xst.Temp,'K',xst.Sal,'ppt'));

nose = 'long';
l = 2.48285; % m; vehicle length (longNose; shortNose: l = 2.24155 m)
xzero = -l/2;

vehicle_coeffs = vehicle_coffs2struct(rho,xzero,nose,'derive');



%% Define optimization parameters and starting point:
%--------------------------------------------------------------------------
% load('~/Documents/MATLAB/MBARI/sim/fminsim/mat/MultiStart_optCoeff_20150716142711.mat')
% load('~/Documents/MATLAB/MBARI/sim/fminsim/mat/coeffcfg/fminsim_coeff_cfg_20150716143602.mat')

% close(fminsim_coeffTable)

fminsim_coeffTable = fminsim_coeffTableUI('last','vehicle_coeffs',vehicle_coeffs,'patch',patch);

vehicle_coeffs = vehicle_coffs2struct(rho,xzero,nose);
fminsim_coeffTable = fminsim_coeffTableUI('last','vehicle_coeffs',vehicle_coeffs,'patch',patch);
%{
optCoeff = {'mass','volume','xg','yg','zg','Mww','Mqq','Muw','Mwdot','Mqdot',...
    'Zwdot', 'Zuw'};

% % coeff = [val0,lb,ub]; where: val0=start value & lb,ub=lower and upper bounds
mass =  [ 153.4671, 145.0, 165.0];
vol =   [ 0.144260585365854, 0.1, 0.2];
xg  =   [ 0.0000000000,   -0.001,  0.001] ;  % m
yg  =   [ 0.0002360000,  -0.0001,  0.0001] ;  % m
zg  =   [ 0.0012627810,  0.00001,  0.01] ;  % m ***0.0067940***

Mww =   [  -4.70,  -75.0,    5.0] ;
Mqq =   [-215.00, -632.0, -190.0] ;
Muw =   [  26.00,    1.0,  105.0] ;
Mwdot = [   5.77,   -2.0,    7.1] ;     % Muq is a dependent
Mqdot = [ -19.60,  -35.0,   -4.0] ;
Zwdot = [-104.00, -126.0,  -35.0] ;
Zuw   = [ -23.95, -104.0,  -15.0] ;

% % Offset optimization parameters (corrections to "input" vec sould always be
% % placed at the end of the coeff vector)
ds1 =    [  0.0000, -Inf, Inf];  % center elev offset
ds2 =    [  0.0000, -Inf, Inf];  % deadband


% % Intialize weight vector for optimization
% %-----------------------------------------------------
w_ = [mass; vol; xg; yg; zg; Mww; Mqq; Muw; Mwdot; Mqdot; Zwdot;...
    Zuw; ds1; ds2];

w0 = w_(:,1);
lb = w_(:,2);
ub = w_(:,3);
%-----------------------------------------------------
%}

%% Evaluate pre-optimization error:
%-----------------------------------------------------
[check,F] = fminTest_sim(w0);

theta_check = check(:,11);
err_strat   = rad2deg(nanmean(sqrt((theta_check - theta).^2)))

% q_check = check(:,5);
% err_strat   = nanmean(sqrt((q_check - q).^2));


% Plot pre-optmization state
%-----------------------------------------------------
close all
pFvars = {'X','M'}; %{'X','Y','Z','K','M','N'};
plot_fminTest(check,F,pFvars,fname)


%% RUN OPTIMIZATION
%--------------------------------------------------------------------------
startTime = datestr(clock); tic

% Define error function:
%-----------------------------------------------------
assert(~isempty(optCoeff),'Select coefficients to optimize before defining the error cost function!')
errorfmin_sim = @(w) nanmean(sqrt((fmin_sim(w,state,input,vehicle_coeffs,n_steps,optCoeff,burnIn) - theta).^2));
errorfun  = @(w) double(errorfmin_sim(w))*(180/pi);


%
% fminsearch
%-----------------------------------------------------
% solvr = 'fminsearch';
% optset  = optimset('MaxIter',300,'TolFun',0.0025,'Display','iter'); % for fminsearch
% [xg,fval,exitflag,output] = fminunc(@(w) errorfun(w),w0, optset); toc

%

% fminunc
%-----------------------------------------------------
% solvr = 'fminunc';
% options = optimoptions('fminunc','MaxIter',250,'Algorithm','quasi-newton',...
%     'TolX',1e-20,'Display','iter-detailed');
% [xg,fval,exitflag,output] = fminunc(errorfun,w0,options); toc


% fmincon
%-----------------------------------------------------
% solvr = 'fmincon';
% options = optimoptions('fmincon','MaxIter',250,'Algorithm','interior-point',...
%     'TolX',1e-20,'TolCon',1e-10,'Display','iter-detailed','UseParallel', true);
% [outCoeff,fval,exitflag,output] = fmincon(errorfun,w0,[],[],[],[],lb,ub,[],options); toc

% patternsearch
%-----------------------------------------------------
% solvr = patternsearch;
% options = psoptimset('MaxIter',500,'Display','iter',...
%     'UseParallel', true, 'CompletePoll', 'on', 'Vectorized', 'off');
% [outCoeff,fval,exitflag,output] = patternsearch(errorfun,w0,[],[],[],[],lb,ub,[],options); toc

% ga
%-----------------------------------------------------
% solvr = ga;
% options = gaoptimset('Display','iter','UseParallel', true, 'Vectorized', 'off');
% [outCoeff,fval,exitflag,output] = ga(errorfun,length(w0),[],[],[],[],lb,ub,[],options); toc
%}

% Global Optimization:
%-----------------------------------------------------
problem = createOptimProblem('fmincon',...
    'objective',@(w) errorfun(w),...
    'x0',w0,'lb',lb,'ub',ub,'options',...
    optimoptions(@fmincon,'Algorithm','interior-point',...
    'UseParallel', true,'Display','off'));
 
% GlobalSearch
%-----------------------------------------------------
% solvr = 'GlobalSearch';
% gs = GlobalSearch('Display','iter');
% [xg,fg,flg,og] = run(gs,problem); toc
 

% % MultiStart
% %-----------------------------------------------------
solvr = 'MultiStart';
ms = MultiStart('StartPointsToRun','bounds',...
    'MaxTime',Inf,'UseParallel',true,'Display','iter');
[xg,fg,flg,og] = run(ms,problem,10); toc


% Evaluate post-optimization error:
%-----------------------------------------------------
outCoeff = xg;
[check,F]  = fminTest_sim(outCoeff);
theta_test = check(:,11);
err_test   = nanmean(sqrt((theta_test - theta).^2));


% Organize optimization inputs/outputs in table and save
%-----------------------------------------------------------------
optVarNames = [optCoeff; 'Error_deg'];
colNames = {'PostOpt_CoeffVal','PreOpt_CoeffVal','AdjFactor','lowerBound','upperBound'};
col1 = [outCoeff; rad2deg(err_test)];
col2 = [w0;  rad2deg(err_strat)];
col3 = col1-col2;
col4 = [lb; NaN];
col5 = [ub; NaN];

optCoeffT = table(col1,col2,col3,col4,col5,'VariableNames',colNames,...
    'RowNames',optVarNames)

patch = v2struct(optVarNames, col1); 

% save
%
outmatfile = [savematfolder solvr '_optCoeff_' searchterm '_' datestr(clock,'yyyymmddHHMMSS')];
save([outmatfile  '.mat'],'optCoeffT','patch')
%}
  

%% Plot post-optmization state
%--------------------------------------------------------------------------
% close all
pFvars = {'X','Y','Z','K','M','N'};
plot_fminTest(check,F,pFvars,fname)

print2pdf(gcf,['~/Desktop/FigTemp/' outmatfile(42:end) ])

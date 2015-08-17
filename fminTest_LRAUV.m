% fminTest_LRAUV.m
clear
clear global
close all
fname = which('fminTest_LRAUV.m'); 

% Gen data:
global n_steps time_step state input optCoeff
timeEval    = 60*5; % sec, evaluation run time
time_step   = 0.4;
n_steps     = fix(timeEval/time_step); %size(ui,1); 
n = 1:n_steps;



% STATE AND INPUT VECTORS:
% state = [u v w p q r xpos ypos zpos phi theta psi]'
state = zeros(length(n),12);
state(:,1) = ones(size(n));    % u
state(:,9) = ones(size(n))*20; % zpos

theta = zeros(length(n(1):n(end)),1);

% input = [ delta_s delta_r Xprop Kprop xg]'
input = zeros(length(n),3);
input(:,3) = ones(length(n),1);


%% Define optimization parameters and starting point:
%--------------------------------------------------------------------------
% load('/Users/benya/Documents/MATLAB/MBARI/sim/fminsim/mat/MultiStart_optCoeff_20150510154941.mat')
fminsim_coeffTable = fminsim_coeffTableUI('last');

%{
% Define optimization parameters and starting point:
%--------------------------------------------------------------------------
optCoeff = {'Mqq','Mww','dCL','xg','zg'};

Mqq = -230.3;        % kg-m2*    Cross-flow drag (Mq|q|)   
Mww = - 62.5;        % kg        Cross-flow drag (-Nv|v|)
Muq = - 61.182063;   % kg-m*
dCL =    4.13;       % n/a       Coef. of Lift Slope
xg  =    0.0;        % m    
zg  =    0.0067940;  % m ***0.0067940***

% Offset optimization parameters
ds1 =    0.0; % center elev offset
ds2 =    0.0; % deadband   

% Intialize weight vector for optimization
%-----------------------------------------------------
inC = [Mqq,Mww,dCL,xg,zg,ds1,ds2]; 
%-----------------------------------------------------
%}

% Define error function:
%--------------------------------------------------------------------------
errorfun  = @(w) nanmean((fmin_sim(w) - theta).^2);

%% RUN SIM

[check,F] = fminTest_sim(inC);
check(end,:) = check(end-1,:);
err_strat   = nanmean((check(:,11) - theta).^2);


%% 
close all
pFvars = {'X','Y','Z','K','M','N'}; 
plot_fminTest(check,F,pFvars,fname)

%% intialCoeffs = vector of coeff starting points
[fw,fval] = fminsearch(@(wi) errorfun(wi),intialCoeffs,'Display','iter');

theta_sim = fmin_sim(x);
p(3) = plot(rad2deg(theta_sim),'linewidth',2);


% Script source water-mark
%------------------------------------------------------------------
uicontrol('Style', 'text','String', [fname ' - ' datestr(clock)],...
    'Units','normalized',...
    'Position', [0 0 1 0.025]);

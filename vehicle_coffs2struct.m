function [ v ] = vehicle_coffs2struct(rho,xzero,nose,varargin)

% Define vehicle coefficients:
%---------------------------------------------------------------------

if nargin > 3
    derive = true;
else
    derive = false;
end


if derive
    v = deriveCoeff(xzero,rho,nose);
end

% Mass Properties:
v.xzero = xzero;% m Origin (calc from bow tip)
v.rho    = rho; % kg/m3
v.g      = 9.806650; % m/s2
v.mass   = 120.7459; % kg Flooded Vehicle mass (based on 242lb dry weight)
% v.movableMass = 31.1391; % kg (68.65lb Jon Erikson SecondaryBatt_V3.3)
v.volume = 0.144260585365854; % m3 (equals 1450 N buoyancy in 1025 kg/m3 water)

% Excludes buoyancy bladder at default settings
v.W = v.mass*v.g ;           % N, Weight
v.B = v.W;                   % N, Buoyancy  B = rho*volume*g;

% Geometric Parameters (Used only by the simulation):
v.xg =  0.0;        % m *** 0.118100***
v.yg =  0.0002360;   % m ***-0.000236***
v.zg =  0.014;       % m *** 0.006794***
v.xb =  0.0;        % m *** 0.118100***
v.yb =  0.0;        % m
v.zb =  0.0;        % m

% Fins:
if strcmpi(nose,'long')
    xfc = -2.1037042;
elseif strcmpi(nose,'short')    
    xfc = -1.8624042;
end
v.xfc     =   xfc-xzero; % m       Fin position wrt origin
v.ARe     =   6.500000;        % n/a     Fin aspect ratios
v.dCL     =   4.130000;        % n/a     Coef. of Lift Slope
v.CDc     =   0.600000;        % n/a     Crossflow Coef. of Drag !!try 0.6!!
v.Cd0     =   0.030000;        % n/a     Min reaction drag coeff
v.ec      =   0.9;             % n/a     Fin efficiency coef
v.Sfin    =   1.15e-2;         % m^2     Fin area

% Mass Properties:
v.Ixx =   3.000000;   % kg-m2     Diagonal inertia tensor
v.Iyy =  41.980233;   % kg-m2     Diagonal inertia tensor 41.980233
v.Izz =  41.980233;   % kg-m2     Diagonal inertia tensor 41.980233

% Thruster parameters:
v.Kprop =  0.23;        % N-m     Propeller Torque ***0.23***

% Stability Derivatives:
v.Xvv =-50.00;     % kg/m; -54.370919
v.Xww =-50.00;     % kg/m; -54.370919

if ~derive
    v.Kpp   = -0.191601;    % kg-m2*  Rolling Resistance             *kg-m2/rad2?
    
    % Added Mass:
    v.Xudot  = -4.876161;   % kg;
    v.Zwdot  = -104.00;       v.Yvdot =  v.Zwdot; % kg;  -126.324739
    
    v.Mwdot  =    5.77;       v.Nvdot = -v.Mwdot; % kg-m; 7.117842
    v.Zqdot  =    v.Mwdot;    v.Yrdot = -v.Mwdot; % kg-m;
    v.Mqdot  =  -33.463086;   v.Nrdot =  v.Mqdot; % kg-m2; -33.463086
    
    v.Kpdot  =    0.000000;   % kg-m2;
    
    % Stability Derivatives:
    v.Xuu = -6.983497;
    v.Xwq =  v.Zwdot;  % kg;
    v.Xqq =  v.Zqdot;  % kg-m;
    v.Xvr = -v.Yvdot;  % kg;
    v.Xrr = -v.Yrdot;  % kg-m;
    
    
    v.Yur =  v.Xudot;       % kg*       Added Mass Cross-term and Fin Lift *kg/rad?   8.719853
    v.Zuq = -v.Xudot;       % kg*       Added Mass Cross-term and Fin Lift *kg/rad? -8.719853***
    v.Ywp = -v.Zwdot;       % kg-m*     Added Mass Cross-term              *kg/rad?
    v.Ypq = -v.Zqdot;       % kg-m      Added Mass Cross-term (-Zqdot)
    
    v.Zww =-601.274653;      % kg/m      Cross-flow Drag
    v.Yvv =   v.Zww;         % kg/m      Cross-flow Drag -601.274653
    v.Zqq =   0.00;          % kg/m      Cross-flow Drag
    v.Yrr =  -v.Zqq;         % n/a*      Cross-flow Drag
    
    v.Zvp =   v.Yvdot;       % kg*       Added Mass Cross-term              *kg/rad?
    v.Zrp =   v.Yrdot;       % kg-m*     Added Mass Cross-term (Yrdot)      *kg/rad?
    
    
    v.Muq = -v.Zqdot; % -61.182063;   % kg-m*     Added Mass Cross-term and Fin Lift *kg-m/rad?
    v.Mrp = (v.Kpdot - v.Nrdot);      % kg-m2*    Added Mass Cross-term          *kg-m2/rad2?
    v.Mvp = -v.Yrdot;                 % kg-m*     Added Mass Cross-term (-Yrdot)   *kg-m/rad?
    v.Mww = -47.7;                    % kg        Cross-flow drag  -58.113144
    v.Nvv = -v.Mww;                   % kg        Cross-flow drag (-Nv|v|)
    
    v.Muw =  24.00; %105.660262;      % kg        Body and Fin Lift and Munk Moment
    v.Nuv = -v.Muw;                   % kg        Body and Fin Lift and Munk Moment
    v.Zuw = -23.954759;               % kg/m      Body Lift Force and Fin Lift
    v.Yuv =  v.Zuw;                   % kg/m      Body Lift Force and Fin Lift ***-23.954759***
    
    v.Mqq = -230.00;              % kg-m2* 	Cross-flow drag (Nr|r|) -632.698957
    v.Nrr =  v.Mqq;               % kg-m2* 	Cross-flow drag (Nr|r|)
    v.Nwp =  v.Zqdot;             % kg-m*   Added Mass Cross-term (Zqdot)    *kg-m/rad?
    v.Npq = -(v.Kpdot - v.Mqdot); % kg-m2*  Added Mass Cross-term          *kg-m2/rad2?
    v.Nur =  v.Yrdot;             % kg-m*   Added Mass Cross-term and Fin Lift *kg-m/rad?
    
end

end

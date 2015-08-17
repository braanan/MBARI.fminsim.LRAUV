function [fminsim_coeffTable] = fminsim_coeffTableUI( varargin )



if ~isempty( varargin )
    
    D = dir('~/Documents/MATLAB/MBARI/sim/fminsim/mat/coeffcfg/');
    D([D.isdir]) = [];
    
    if strcmpi(varargin(1), 'last')
        
        [~,last] = max([D.datenum]);
        tmp = D(last);
        fname = ['~/Documents/MATLAB/MBARI/sim/fminsim/mat/coeffcfg/' tmp.name];
        
    elseif strcmpi(varargin(1), 'load')
        
        fname = uigetfile('~/Documents/MATLAB/MBARI/sim/fminsim/mat/coeffcfg/',...
            'Select configuration file:' );
        
    elseif strcmpi(varargin(1), 'original')
        
        fname = '~/Documents/MATLAB/MBARI/sim/fminsim/mat/coeffcfg/Original/fminsim_coeffTable_OriginalCfg.mat';
        
    end
    display(['[fminsim_coeffTableUI]: Loaded coeff configuration from: ' fname])
    load(fname)
    
    
    % patch starting values from struct
    if numel(varargin)>1
        % patch from vehicle_coeffs struct
        if any(strcmpi(varargin,'vehicle_coeffs'))
            
            vc = varargin{find(strcmpi(varargin,'vehicle_coeffs'))+1};
            dname = fieldnames(vc);
            
            for k=1:numel(dname)
                idx = strcmp(d(:,1),dname{k});
                if any(idx)
                    d{idx,2} = double(vc.(dname{k}));
                end
            end
            display('[fminsim_coeffTableUI]: Set coeff values to vehicle_coeffs')    
        end
        
        % patch from optimization output struct
        if any(strcmpi(varargin,'patch'))
            ptch = varargin{find(strcmpi(varargin,'patch'))+1};
            dname = ptch.optVarNames(1:end-1);
            dval  = double(ptch.col1(1:end-1));
            
            for k=1:numel(dname)
                idx = ~cellfun('isempty',strfind(d(:,1),dname{k}));
                d{idx,2} = dval(k);
            end
            
            display('[fminsim_coeffTableUI]: Patched coeff values')    
        end
    
    end
    
else
    
    % Define the data
    %---------------------------------------------------------------------
    k=0;
    k=k+1; d(k,:) = {'rho',   1025.0000, 1000.00, 1050.00, false, '', 'Seawater Density'};
    k=k+1; d(k,:) = {'mass',   153.4671,  133.50,  173.50, false, '', 'Flooded Vehicle mass'};        
    k=k+1; d(k,:) = {'volume', 153.4671/1025, 0.10, 0.20, false, '', 'Vehicle Vol'} ;                     
    k=k+1; d(k,:) = {'W', 1504.99, 1405.00, 1605.00, true, '', 'Weight (N)'}; 
    k=k+1; d(k,:) = {'B', 1504.99, 1405.00, 1605.00, true, '', 'Buoyancy (N)'};
    
    % Geometric Parameters (Used only by the simulation):
    k=k+1; d(k,:) =  {'xg', 0.0000000, -0.01 , 0.01 , true, '', 'Geometric Parameter'}; % m *** 0.118100***
    k=k+1; d(k,:) =  {'yg', 0.0002360, -0.001, 0.001, true, '', 'Geometric Parameter'}; % m ***-0.000236***
    k=k+1; d(k,:) =  {'zg', 0.0140000,  0.0  , 0.1  , true, '', 'Geometric Parameter'}; % m *** 0.006794***
    k=k+1; d(k,:) =  {'xb', 0.0000000, -0.01 , 0.01 , false, '', 'Geometric Parameter'}; % m *** 0.118100***
    k=k+1; d(k,:) =  {'yb', 0.0000000, -0.001, 0.001, false, '', 'Geometric Parameter'}; % m
    k=k+1; d(k,:) =  {'zb', 0.0000000, -0.01 , 0.01 , false, '', 'Geometric Parameter'}; % m
    
    % Mass Properties:
    k=k+1; d(k,:) = {'Ixx', 3.000000,  1.00,  5.00,  false, '', 'Diagonal inertia tensor'};       % kg-m2      Diagonal inertia tensor
    k=k+1; d(k,:) = {'Iyy', 41.980233, 10.00, 75.00, false, '', 'Diagonal inertia tensor'};   % kg-m2     Diagonal inertia tensor 41.980233
    k=k+1; d(k,:) = {'Izz', 41.980233, 10.00, 75.00, false, '', 'Diagonal inertia tensor'};   % kg-m2     Diagonal inertia tensor 41.980233
    
    % Added Mass:
    k=k+1; d(k,:)  = {'Xudot',  -4.876161,   -9,  -1,  true, 'Yur, -Zuq', 'Added Mass'};   % kg;
    k=k+1; d(k,:)  = {'Zwdot',-104.000000, -150, -35,  true, '-Yvdot, Xwq, Xvr, Ywp, -Zvp', 'Added Mass'}; % kg -126.324739
    k=k+1; d(k,:)  = {'Mwdot',   5.770000,   -2,   8,  true, 'Zqdot, -Yrdot, -Nvdot, Xqq, Xrr, -Ypq, -Zrp, -Muq, -Mvp','Added Mass'};
    k=k+1; d(k,:)  = {'Mqdot', -33.463086,  -45,  -4,  true, 'Nrdot', 'Added Mass'};
    k=k+1; d(k,:)  = {'Kpdot',   0.080000,   -2,   2,  true, '','Added Mass'};   % kg-m2;
    
    % Stability Derivatives:
    k=k+1; d(k,:) = {'Kpp',  -0.191601,  -0.35,  -0.05, true, '', 'Rolling Resistance'};    % kg-m2*  Rolling Resistance             *kg-m2/rad2?
    k=k+1; d(k,:) = {'Xuu',  -6.983497, -12.00,  -2.00, true, '', 'Cross-flow Drag'};
    k=k+1; d(k,:) = {'Xvv', -50.000000, -60.00, -40.00, true, '', 'Cross-flow Drag'};      % kg/m; -54.370919
    k=k+1; d(k,:) = {'Xww', -50.000000, -60.00, -40.00, true, '', 'Cross-flow Drag'};      % kg/m; -54.370919
    k=k+1; d(k,:) = {'Zww',-504.000000,-601.00,-131.00, true, ' Yvv', 'Cross-flow Drag'};      % kg/m      Cross-flow Drag
    k=k+1; d(k,:) = {'Zqq',   0.000000,  -1.00,   0.00, true, '-Yrr', 'Cross-flow Drag'};      % kg/m      Cross-flow Drag
    k=k+1; d(k,:) = {'Zuw', -23.954759, -32.00, -12.00, true, ' Yuv', 'Cross-flow Drag'};      % kg        Cross-flow drag  -58.113144
    k=k+1; d(k,:) = {'Mww', -47.700000, -58.00,   2.00, true, '-Nvv', 'Cross-flow Drag'};      % kg        Cross-flow drag  -58.113144
    k=k+1; d(k,:) = {'Muw',  24.00,      20.00, 105.66, true, '-Nuv', 'Cross-flow Drag'};      % Body and Fin Lift and Munk Moment
    k=k+1; d(k,:) = {'Mqq',-215.00,    -632.69,-190.00, true, ' Nrr', 'Cross-flow Drag'};      % Body and Fin Lift and Munk Moment
    
    % Fins:
    k=k+1; d(k,:) = {'ARe', 6.500000, 6.500000, 6.500000, false, '', 'Fin aspect ratios'} ;     % n/a     Fin aspect ratios
    k=k+1; d(k,:) = {'dCL', 4.130000, 4.130000, 4.130000, false, '', 'Coef. of Lift Slope'} ;   % n/a     Coef. of Lift Slope
    k=k+1; d(k,:) = {'CDc', 0.600000, 0.600000, 0.600000, false, '', 'Crossflow Coef. of Drag'};% n/a     Crossflow Coef. of Drag !!try 0.6!!
    k=k+1; d(k,:) = {'Cd0', 0.030000, 0.030000, 0.030000, false, '', 'Min reaction drag coeff'};% n/a     Min reaction drag coeff
    k=k+1; d(k,:) = {'ec' , 0.900000, 0.900000, 0.900000, false, '', 'Lift Slope Parameter'};
    k=k+1; d(k,:) = {'Sfin',1.15e-2, 1.15e-2, 1.15e-2,    false, '', 'Fin Planeform Area'};     % m^2     Fin area
    
    k=k+1; d(k,:) = {'EleOffset', 0.0, -Inf, Inf, true, '', 'Elevator offset'};
    k=k+1; d(k,:) = {'EleDead'  , 0.0, -Inf, Inf, true, '', 'Elevator deadband'};
    
    % save('~/Documents/MATLAB/MBARI/sim/fminsim/mat/coeffcfg/Original/fminsim_coeffTable_OriginalCfg.mat','d')
end






% Launch GUI:
%--------------------------------------------------------------------------
fminsim_coeffTable = figure;
set(fminsim_coeffTable, 'Units','normalized','Position',[0.25 0.25 0.375 0.625]);
% Column names and column format
%--------------------------------------------------------
columnname   = {'Coeff','Value','LowerBound','UpperBound','Fixed/Adj','Dependencies','Comments'};
columnformat = {'char','numeric','numeric','numeric','logical','char','char'};

% Create the uitable
%--------------------------------------------------------
t = uitable('Data', d,...
    'ColumnName', columnname,...
    'ColumnFormat', columnformat,...
    'ColumnEditable', [false true true true true true true],...
    'ColumnWidth',{'auto','auto','auto','auto',60,100,175},...
    'CellEditCallback', @TableCallback);

% Set width and height
t.FontSize = 12.5;
t.Units = 'normalized';
t.Position(1) = 0.025;
t.Position(2) = 0.075;
t.Position(3) = 0.95;   %t.Extent(3);
t.Position(4) = 0.875;  %t.Extent(4);
% Export push button
%--------------------------------------------------------
bsp = uicontrol('String','Export to workspace',...
    'Units','normalized',...
    'Position',[0.55 0.015 0.2 0.05],...
    'Callback',@pushbutton_Callback);

% Save push button
%--------------------------------------------------------
bsp2 = uicontrol('String','Save config to file',...
    'Units','normalized',...
    'Position',[0.765 0.015 0.2 0.05],...
    'Callback',@savebutton_Callback);

set(fminsim_coeffTable, 'HandleVisibility', 'off');



% CallBack functions for GUI:
%--------------------------------------------------------------------------

    function TableCallback(t,callbackdata)
        
        val = (callbackdata.EditData);
        r = callbackdata.Indices(1);
        c = callbackdata.Indices(2);
        
        if any(c==2:4)
            val = str2double(val);
        end
        t.Data{r,c} = val;
        
    end

    function pushbutton_Callback(bsp, eventdata)
        % hObject    handle to pushbutton1 (see GCBO)
        % eventdata  reserved - to be defined in a future version of MATLAB
        % handles    structure with handles and user data (see GUIDATA)
        
        T = fminsim_coeffTable.Children;
        dat = T(3).Data;
        ind = cell2mat(dat(:,5));
        
        assignin('base', 'optCoeff', dat(ind,1));
        assignin('base', 'w0', cell2mat(dat(ind,2)));
        assignin('base', 'lb', cell2mat(dat(ind,3)));
        assignin('base', 'ub', cell2mat(dat(ind,4)));
        display('[fminsim_coeffTableUI]: Exported data to workspace!')
    end

    function savebutton_Callback(bsp2, eventdata)
        
        T = fminsim_coeffTable.Children;
        d = T(3).Data;
        
        fold  = '~/Documents/MATLAB/MBARI/sim/fminsim/mat/coeffcfg/';
        fsave = [fold 'fminsim_coeff_cfg_' datestr(clock,'yyyymmddHHMMSS') '.mat'];
        display([datestr(clock) ' [fminsim_coeffTableUI]: Saved coeff configuration to: ' fsave])
        save(fsave, 'd')
    end


end
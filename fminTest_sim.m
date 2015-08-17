function  [simlog,F]  = fminTest_sim(w)

global state input vehicle_coeffs n_steps optCoeff burnIn

% vehicle_coeffs = vehicle_coffs2struct;

% Adjust weights
%---------------------------------------------------------------------
if numel(optCoeff)>2
    for c = 1:numel(optCoeff)
        vehicle_coeffs.(optCoeff{c}) = w(c);
    end
end

time_step = 0.4;
F = struct;
x = state(1,:);
simlog = zeros(n_steps,12);

% Run simulation
%---------------------------------------------------------------------
for k = 1:n_steps-1
    
    
    % Set some vars constant
    % x(1)  = state(k,1);
    % x(4)  = state(k,4);
    % x(10) = state(k,10);
    
    u_in = input(k,:);
    
    u_in(1) = u_in(1) + w(end-1);
    u_in(u_in(:,1) < 0,1) = u_in(u_in(:,1) < 0,1) - w(end);
    u_in(u_in(:,1) > 0,1) = u_in(u_in(:,1) > 0,1) + w(end);
    
    % Calc next step
    [xdot,~,X,Y,Z,K,M,N] = fminTest_lrauvsim(x,u_in,vehicle_coeffs);
    
    % Log step data
    simlog(k,:) = x;
    F.X(k,:) = X; F.Y(k,:) = Y; F.Z(k,:) = Z; % forces
    F.K(k,:) = K; F.M(k,:) = M; F.N(k,:) = N; % moments
    
    % RUNGE-KUTTA APPROXIMATION to calculate new states
    % NOTE: ideally, should be approximating ui values for k2,k3
    k1_vec = xdot;
    k2_vec = fminTest_lrauvsim(x+(0.5.*time_step.*k1_vec)', ((input(k,:)+input(k+1,:))./2), vehicle_coeffs) ;
    k3_vec = fminTest_lrauvsim(x+(0.5.*time_step.*k2_vec)', ((input(k,:)+input(k+1,:))./2), vehicle_coeffs) ;
    k4_vec = fminTest_lrauvsim(x+(time_step.*k3_vec)', input(k+1,:), vehicle_coeffs) ;
    x = x + time_step/6.*(k1_vec +2.*k2_vec +2.*k3_vec +k4_vec)';
    
end;

% Write
%---------------------------------------------------------------------
F.names.X = {'X_{HS}','Xprop','Xuu','Xvv','Xww','Xvr','Xwq','Xrr','Xqq','m','Fins'};
F.names.Y = {'Y_{HS}','Yvv','Yuv','Yur','Yrr','Ywp','m','Fins'};
F.names.Z = {'Z_{HS}','Zww','Zqq','Zuq','Zuw','Zvq','m','Fins'};
F.names.K = {'K_{HS}','Kpp','m','Kprop','Fins'};
F.names.M = {'M_{HS}','Mww','Mqq','Muw','Muq','Mpr','I','m','Fins'};
F.names.N = {'N_{HS}','Nvv','Nrr','Nuv','Nur','Npq','I','m','Fins'};

simlog(end,:) = simlog(end-1,:);
simlog(1:burnIn,:) = NaN;

end

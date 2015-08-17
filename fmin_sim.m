function sim_  = fmin_sim(w,state,input,vehicle_coeffs,n_steps,optCoeff,burnIn)

% vehicle_coeffs = vehicle_coffs2struct;

% Adjust weights
%---------------------------------------------------------------------
if numel(optCoeff)>2
    for c = 1:numel(optCoeff)-2
        vehicle_coeffs.(optCoeff{c}) = w(c);
    end
end

time_step = 0.4;
simlog = zeros(n_steps,1);
x = state(1,:);

% Run simulation
%---------------------------------------------------------------------
for k = 1:n_steps
    
    % Set some vars constant
    % x(1)  = state(k,1);
    % x(4)  = state(k,4);
    % x(10) = state(k,10);
    
    u_in = input(k,:);
    
    u_in(1) = u_in(1) + w(end-1);
    u_in(u_in(:,1) < 0,1) = u_in(u_in(:,1) < 0,1) - w(end);
    u_in(u_in(:,1) > 0,1) = u_in(u_in(:,1) > 0,1) + w(end);
    
    % Calc next step
    [xdot,~] = fmin_lrauvsim(x,u_in,vehicle_coeffs);
    
    % Log previous step data 
    simlog(k,:) = x(11); % theta
    % simlog(k,:) = x(5);  % q
    
    % RUNGE-KUTTA APPROXIMATION to calculate new states
    % NOTE: ideally, should be approximating ui values for k2,k3
    k1 = xdot;
    k2 = fmin_lrauvsim( x+(0.5.*time_step.*k1)', ((input(k,:)+input(k+1,:))./2), vehicle_coeffs ) ;
    k3 = fmin_lrauvsim( x+(0.5.*time_step.*k2)', ((input(k,:)+input(k+1,:))./2), vehicle_coeffs ) ;
    k4 = fmin_lrauvsim( x+(time_step.*k3)', input(k+1,:), vehicle_coeffs ) ;
    
    x = x + time_step/6.*(k1 +2.*k2 +2.*k3 +k4)';
    
end;

% Write
%---------------------------------------------------------------------
simlog(end,:) = simlog(end-1,:);
simlog(1:burnIn,:) = NaN;

% theta_sim = simlog;
sim_ = simlog;
end
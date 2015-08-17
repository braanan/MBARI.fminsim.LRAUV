% fminTest_lrauvsim.m  - LRAUV Simulator Testground 

% Returns the time derivative of the state vector
% Following REMUS sim by Timothy Prestero, MIT/WHOI 2001 
% Last modified May 4, 2015 by Ben Raanan


function [ACCELERATIONS,FORCES,Xcomp,Ycomp,Zcomp,Kcomp,Mcomp,Ncomp] = fminTest_lrauvsim(x,u_in,c)

% TERMS
% ---------------------------------------------------------------------
% STATE VECTOR:
% x = [u v w p q r xpos ypos zpos phi theta psi]'
%  Body-referenced Coordinates
%  u            = Surge velocity            [m/sec]
%  v            = Sway velocity             [m/sec]
%  w            = Heave velocity            [m/sec]
%  p            = Roll rate                 [rad/sec]
%  q            = Pitch rate                [rad/sec]
%  r            = Yaw rate                  [rad/sec]
%  Earth-fixed coordinates
%  xpos         = Position in x-direction   [m]
%  ypos         = Position in y-direction   [m]
%  zpos         = Position in z-direction   [m]
%  phi          = Roll angle                [rad]
%  theta        = Pitch angle               [rad]
%  psi          = Yaw angle                 [rad]
%
% INPUT VECTOR
% ui = [delta_s delta_r]'
%  Control Fin Angles
%  delta_s  = angle of stern planes         [rad]
%  delta_r  = angle of rudder planes        [rad]


% Define vehicle coefficients (see vehicleCoeffs_2_struct.m):
%---------------------------------------------------------------------
% Mass Properties:
rho = c.rho; m = c.mass; Wp = c.W;	Bp = c.B; % g = c.g; volume = c.volume;
% Excludes buoyancy bladder at default settings

% Geometric Parameters (Used only by the simulation):
Xgp = c.xg;   Xbp = c.xb;
Ygp = c.yg;   Ybp = c.yb;
Zgp = c.zg;   Zbp = c.zb;

% Fins:
ARe = c.ARe; dCL = c.dCL; CDc = c.CDc; 
Cd0 = c.Cd0; ec  = c.ec; Sfin = c.Sfin;
xfc = c.xfc;

% Mass Properties:
Ixx =  c.Ixx; Iyy = c.Iyy; Izz = c.Izz;

% Thruster parameters:
Kpp   = c.Kpp; 

% Added Mass:
Yvdot = c.Yvdot; Zwdot = c.Zwdot; Mwdot = c.Mwdot; Nvdot = c.Nvdot;
Yrdot = c.Yrdot; Zqdot = c.Zqdot; Mqdot = c.Mqdot; Nrdot = c.Nrdot;
Xudot = c.Xudot; Kpdot = c.Kpdot; % Kvdot = c.Kvdot; Ypdot = c.Ypdot;

% Stability Derivatives:
Xuu = c.Xuu; Xwq = c.Xwq; Xqq = c.Xqq; Xrr = c.Xrr; Xvr = c.Xvr; Xvv = c.Xvv; Xww = c.Xww;
Yur = c.Yur; Ywp = c.Ywp; Yvv = c.Yvv; Yrr = c.Yrr; Yuv = c.Yuv; % Ypq = c.Ypq;
Zqq = c.Zqq; Zww = c.Zww; Zuq = c.Zuq; Zvp = c.Zvp; Zuw = c.Zuw; % Zrp = c.Zrp;
Mqq = c.Mqq; Muq = c.Muq; Mww = c.Mww; Muw = c.Muw; Mrp = c.Mrp; % Mvp = c.Mvp;
Nvv = c.Nvv; Nuv = c.Nuv; Nrr = c.Nrr; Npq = c.Npq; Nur = c.Nur; % Nwp = c.Nwp;
%}

% Form the generalized mass matrix and invert it:
%---------------------------------------------------------------------
[ Minv ] = invMassMat_inline;

% Get and check state variables and control inputs
%---------------------------------------------------------------------
% Get state variables
u   = x(1) ; v  = x(2) ; w  = x(3) ; p  = x(4) ; q  = x(5) ; r  = x(6);
phi = x(10) ; theta  = x(11) ; psi  = x(12) ;

% Get control inputs
elev_ang = u_in(1); rud_ang = u_in(2);
Xprop    = u_in(3); Kprop   = u_in(4);

% Initialize elements of coordinate system transform matrix
%---------------------------------------------------------------------
c1 = cos(phi); c2 = cos(theta); c3 = cos(psi);
s1 = sin(phi); s2 = sin(theta); s3 = sin(psi);
t2 = tan(theta);

% Get fin forces and moments
%---------------------------------------------------------------------
[ F1, F2, F3, F4, M1, M2, M3, M4 ] = robsFins_inline;

% [bodyLift, bodyMoment] = bodyLiftMoment(u, w);
% [ Yr, Zs, Ms, Nr ] = fins( rho, u, v, w, q, r, delta_r, delta_s );

% Set total forces from equations of motion
%--------------------------------------------------------------------------

% Surge:
% X = - (Wp-Bp)*s2 + Xuu*u*abs(u) + Xprop...
%     + m*(v*r - w*q + Xgp*(q*q + r*r) - Ygp*p*q - Zgp*p*r) ...
%     + F1(1) + F2(1) + F3(1) + F4(1)...
%     + Xvv*v*v + Xww*w*w + Xvr*v*r + Xwq*w*q + Xrr*r*r + Xqq*q*q;
Xcomp = surge;
X = sum(Xcomp);

% Sway:
% Y = (Wp-Bp)*c2*s3 + Yvv*v*abs(v)...
%     + Yuv*u*v + Yur*u*r + Yrr*r*abs(r) + Ywp*w*p...
%     + m*(w*p - u*r + Ygp*(r*r+p*p) -Zgp*q*r - Xgp*p*q)...
%     + F1(2) + F2(2) + F3(2) + F4(2);
Ycomp = sway; Y = sum(Ycomp);

% Sway:
% Z = (Wp-Bp)*c2*c3 + Zww*w*abs(w) + Zqq*q*abs(q)...
%     + Zuq*u*q + Zuw*u*w + Zvp*v*p...
%     + m*(u*q - v*p + Zgp*(p*p + q*q) - Xgp*p*r - Ygp*q*r)...
%     + F1(3) + F2(3) + F3(3) + F4(3) ; 
Zcomp = heave; Z = sum(Zcomp);

% Roll:
% K = -(Ygp*W-Ybp*B)*cos(theta)*cos(phi) - (Zgp*W-Zbp*B)*cos(theta)*sin(phi) ...
%     + Kpp*p*abs(p) - (Izz-Iyy)*q*r - (m*Zgp)*w*p + (m*Zgp)*u*r...
%     + Kprop + M1(1) + M2(1) + M3(1) + M4(1);
Kcomp = roll; K = sum(Kcomp);

% Pitch:
% M = -(Zgp*Wp - Zbp*Bp)*s2 - (Xgp*Wp - Xbp*Bp)*c2*c1...
%     + Mww*w*abs(w) + Mqq*q*abs(q) ...
%     + Muw*u*w + Muq*u*q + Mpr*p*r...
%     + (Izz - Ixx)*p*r...
%     - m*(Zgp*(w*q - v*r) + Xgp*(u*q - v*p))...
%     + M1(2) + M2(2) + M3(2) + M4(2) ; 
Mcomp = pitch; M = sum(Mcomp);

% Yaw:
% N = (Ygp*Wp - Ybp*Bp)*s2 + (Xgp*Wp - Xbp*Bp)*c2*s1...         
%     + Nvv*v*abs(v) + Nrr*r*abs(r) + Nuv*u*v ...
%     + Nur*u*r + Npq*p*q...
%     + (Ixx - Iyy)*p*q...
%     - m*Xgp*(u*r - w*p) + m*Ygp*(w*q - v*r)...
%     + M1(3) + M2(3) + M3(3) + M4(3);
Ncomp = yaw; N  = sum(yaw);


FORCES = [X Y Z K M N]' ;

ACCELERATIONS = ...
    [Minv(1,1)*X+Minv(1,2)*Y+Minv(1,3)*Z+Minv(1,4)*K+Minv(1,5)*M+Minv(1,6)*N
    Minv(2,1)*X+Minv(2,2)*Y+Minv(2,3)*Z+Minv(2,4)*K+Minv(2,5)*M+Minv(2,6)*N
    Minv(3,1)*X+Minv(3,2)*Y+Minv(3,3)*Z+Minv(3,4)*K+Minv(3,5)*M+Minv(3,6)*N
    Minv(4,1)*X+Minv(4,2)*Y+Minv(4,3)*Z+Minv(4,4)*K+Minv(4,5)*M+Minv(4,6)*N
    Minv(5,1)*X+Minv(5,2)*Y+Minv(5,3)*Z+Minv(5,4)*K+Minv(5,5)*M+Minv(5,6)*N
    Minv(6,1)*X+Minv(6,2)*Y+Minv(6,3)*Z+Minv(6,4)*K+Minv(6,5)*M+Minv(6,6)*N
    c3*c2*u + (c3*s2*s1-s3*c1)*v + (s3*s1+c3*c1*s2)*w
    s3*c2*u + (c1*c3+s1*s2*s3)*v + (c1*s2*s3-c3*s1)*w
      -s2*u +            c2*s1*v +            c1*c2*w
          p +            s1*t2*q +            c1*t2*r
       c1*q -               s1*r
    s1/c2*q +            c1/c2*r] ;



% Simulate lift/drag forces and moments applied by fins
%--------------------------------------------------------------------------
    function [ F1, F2, F3, F4, M1, M2, M3, M4 ] = robsFins_inline()
        
        % robsFins: Simulate lift/drag forces and moments applied to each 
        %           of LRAUV fins.
        % AUTHOR:   Rob McEwen, MBARI - 2008/2
        
        % Initialize global variables
        %------------------------------------------------------------------
        stall_angle    =   30.000000*pi/180; % arcdeg
        
        % Position of fins from vehicle center:
        % colName = {'lowerRud','portElev','upperRud','stbdElev'};
        % rowName = {'X','Y','Z'};
        xi = 1 ; yi = 2 ; zi = 3 ; % dimention indecies
        % finPos  = [ -0.633, -0.633, -0.633, -0.633 ;
        %              0.012, -0.152,  0.012,  0.152 ;
        %             -0.152,  0.000,  0.152,  0.000 ] ;  % m
        
        
        finPos  = [     xfc,     xfc,     xfc,     xfc ;
                     0.0000, -0.2421,  0.0000,  0.2421 ;
                    -0.2421,  0.0000,  0.2421,  0.0000 ] ;  % m
        % Get and check state variables and control inputs
        %------------------------------------------------------------------
        % Get state variables
        % u   = x(1) ; v  = x(2) ; w  = x(3); 
        % p   = x(4) ; q  = x(5) ; r  = x(6);
        
        
        % Initialize elements of coordinate system transform matrix
        %------------------------------------------------------------------
        fs1 = sin(rud_ang); fs2 = sin(elev_ang);
        fc1 = cos(rud_ang); fc2 = cos(elev_ang);
        
        
        % Calculate fin velocities in body coords
        %------------------------------------------------------------------
        v1  = [ u + q*finPos(zi,1) - r*finPos(yi,1) ;
                v - p*finPos(zi,1) + r*finPos(xi,1) ;
                w + p*finPos(yi,1) - q*finPos(xi,1) ] ;
        
        v2  = [ u + q*finPos(zi,2) - r*finPos(yi,2) ;
                v - p*finPos(zi,2) + r*finPos(xi,2) ;
                w + p*finPos(yi,2) - q*finPos(xi,2) ] ;
        
        v3 	= [ u + q*finPos(zi,3) - r*finPos(yi,3) ;
                v + r*finPos(xi,3) - p*finPos(zi,3) ;
                w + p*finPos(yi,3) - q*finPos(xi,3) ] ;
        
        v4  = [ u + q*finPos(zi,4) - r*finPos(yi,4) ;
                v - p*finPos(zi,4) + r*finPos(xi,4) ;
                w + p*finPos(yi,4) - q*finPos(xi,4) ] ;
        
        
        % Now get angle of attack for each fin
        %------------------------------------------------------------------
        norm_v1 = sqrt( v1(1)*v1(1) + v1(2)*v1(2) + v1(3)*v1(3) );
        if (norm_v1 < 0.001)
            alpha1 = 0.0;
        else
            alpha1 =  rud_ang - v1(2)/v1(1); 
        end
        
        norm_v2 = sqrt( v2(1)*v2(1) + v2(2)*v2(2) + v2(3)*v2(3) );
        if (norm_v2 < 0.001)
            alpha2 = 0.0;
        else
            alpha2 =  elev_ang + v2(3)/v2(1); 
        end
        
        norm_v3 = sqrt( v3(1)*v3(1) + v3(2)*v3(2) + v3(3)*v3(3) );
        if (norm_v3 < 0.001)
            alpha3=0.0;
        else
            alpha3 = rud_ang - v3(2)/v3(1); 
        end
        
        norm_v4 = sqrt( v4(1)*v4(1) + v4(2)*v4(2) + v4(3)*v4(3) );
        if (norm_v4 < 0.001)
            alpha4=0.0;
        else
            alpha4 = elev_ang + v4(3)/v4(1); 
        end
        
        % lift and drag coefficients */
        CDC = CDc/ARe;
        
        % Note that if stall angle is exceeded: NO LIFT */
        if (abs(alpha1) < stall_angle)
            CL1 = dCL*alpha1 + CDC*alpha1*abs(alpha1);
        else CL1 = 0. ;
        end
        
        if (abs(alpha2) < stall_angle)
            CL2 = dCL*alpha2 + CDC*alpha2*abs(alpha2);
        else CL2 = 0. ;
        end
        
        if (abs(alpha3) < stall_angle)
            CL3 = dCL*alpha3 + CDC*alpha3*abs(alpha3);
        else CL3 = 0. ;
        end
        
        if (abs(alpha4) < stall_angle)
            CL4 = dCL*alpha4 + CDC*alpha4*abs(alpha4);
        else CL4 = 0. ;
        end
        
        aa = 1.0/(pi*ARe*ec);
        
        CD1 = Cd0 + aa*CL1*CL1;
        CD2 = Cd0 + aa*CL2*CL2;
        CD3 = Cd0 + aa*CL3*CL3;
        CD4 = Cd0 + aa*CL4*CL4;
        
        % lift and drag forces, in flow coords... */
        %---------------------------------------------------------------------
        cons = (rho*Sfin)/2.0;
        
        LW1 = cons*norm_v1*norm_v1*CL1;     % positive when the lift 
        LW2 = cons*norm_v2*norm_v2*CL2;     % vector is close to normal 
        LW3 = cons*norm_v3*norm_v3*CL3;     % vector
        LW4 = cons*norm_v4*norm_v4*CL4;
        
        DW1 = cons*norm_v1*norm_v1*CD1;     % always positive 
        DW2 = cons*norm_v2*norm_v2*CD2;
        DW3 = cons*norm_v3*norm_v3*CD3;
        DW4 = cons*norm_v4*norm_v4*CD4;
        
        LF1 = LW1*cos(alpha1) + DW1*sin(alpha1);    % force in the fin 
        LF2 = LW2*cos(alpha2) + DW2*sin(alpha2);    % normal direction
        LF3 = LW3*cos(alpha3) + DW3*sin(alpha3);
        LF4 = LW4*cos(alpha4) + DW4*sin(alpha4);
        
        DF1 = -LW1*sin(alpha1) + DW1*cos(alpha1);   % force in the fin-aft 
        DF2 = -LW2*sin(alpha2) + DW2*cos(alpha2);   % direction
        DF3 = -LW3*sin(alpha3) + DW3*cos(alpha3);
        DF4 = -LW4*sin(alpha4) + DW4*cos(alpha4);
        
        % Finally, transform into the body frame */
        %---------------------------------------------------------------------
        F1  = [ -LF1*fs1 + (-DF1)*fc1 ;
                 LF1*fc1 + (-DF1)*fs1 ;
                 0.0                  ] ;
        
        F2  = [ -LF2*fs2 + (-DF2)*fc2 ;
                 0.0                  ;
                -LF2*fc2 - (-DF2)*fs2 ] ;
        
        F3	= [ -LF3*fs1 + (-DF3)*fc1 ;
                 LF3*fc1 + (-DF3)*fs1 ;
                 0.0                  ] ;
        
        F4  = [ -LF4*fs2 + (-DF4)*fc2 ;
                 0.0                  ;
                -LF4*fc2 - (-DF4)*fs2 ] ;
        
        % moments induced by these forces */
        M1  = [ finPos(yi,1)*F1(3) - finPos(zi,1)*F1(2) ;
               -finPos(xi,1)*F1(3) + finPos(zi,1)*F1(1) ;
                finPos(xi,1)*F1(2) - finPos(yi,1)*F1(1) ] ;
        
        M2  = [ finPos(yi,2)*F2(3) - finPos(zi,2)*F2(2) ;
               -finPos(xi,2)*F2(3) + finPos(zi,2)*F2(1) ;
                finPos(xi,2)*F2(2) - finPos(yi,2)*F2(1) ] ;
        
        M3  = [ finPos(yi,3)*F3(3) - finPos(zi,3)*F3(2) ;
               -finPos(xi,3)*F3(3) + finPos(zi,3)*F3(1) ;
                finPos(xi,3)*F3(2) - finPos(yi,3)*F3(1) ] ;
        
        M4  = [ finPos(yi,4)*F4(3) - finPos(zi,4)*F4(2) ;
               -finPos(xi,4)*F4(3) + finPos(zi,4)*F4(1) ;
                finPos(xi,4)*F4(2) - finPos(yi,4)*F4(1) ] ;
    end

% Generalized mass matrix inverse:
%--------------------------------------------------------------------------
    function [ Minv ] = invMassMat_inline()
        MM    = zeros(6,6);
        MM(1,:) = [ m-Xudot,          0,           0,         0,        m*Zgp,      -m*Ygp ];
        MM(2,:) = [       0,    m-Yvdot,           0,     -m*Zgp,           0, m*Xgp-Yrdot ];
        MM(3,:) = [       0,          0,     m-Zwdot,      m*Ygp, -m*Xgp-Zqdot,          0 ];
        MM(4,:) = [       0,      -m*Zgp,        m*Ygp, Ixx-Kpdot,           0,          0 ];
        MM(5,:) = [    m*Zgp,          0, -m*Xgp-Mwdot,         0,   Iyy-Mqdot,          0 ];
        MM(6,:) = [   -m*Ygp, m*Xgp-Nvdot,           0,         0,           0,  Izz-Nrdot ];
        
        Minv = inv(MM);
    end
    

% Total forces and moments from equations of motion
%--------------------------------------------------------------------------
    function Xcomp = surge()
        Xcomp(1) = - (Wp-Bp)*s2; 
        Xcomp(2) = Xprop;       Xcomp(3) = Xuu*u*abs(u); 
        Xcomp(4) = Xvv*v*v;     Xcomp(5) = Xww*w*w; 
        Xcomp(6) = Xvr*v*r;     Xcomp(7) = Xwq*w*q;
        Xcomp(8) = Xrr*r*r;     Xcomp(9) = Xqq*q*q;
        Xcomp(10) = m*(v*r - w*q + Xgp*(q*q + r*r) - Ygp*p*q - Zgp*p*r);
        Xcomp(11) =  F1(1) + F2(1) + F3(1) + F4(1);
    end

    function Ycomp = sway()
        Ycomp(1) = (Wp-Bp)*c2*s3;    Ycomp(2) = Yvv*v*abs(v);
        Ycomp(3) =  Yuv*u*v;         Ycomp(4) = Yur*u*r; 
        Ycomp(5) =  Yrr*r*abs(r);    Ycomp(6) = Ywp*w*p; 
        Ycomp(7) =  m*(w*p - u*r + Ygp*(r*r+p*p) -Zgp*q*r - Xgp*p*q);
        Ycomp(8) =  F1(2) + F2(2) + F3(2) + F4(2);
    end

    function Zcomp = heave()
        Zcomp(1) = (Wp-Bp)*c2*c3;   Zcomp(2) = Zww*w*abs(w);
        Zcomp(3) = Zqq*q*abs(q);    Zcomp(4) = Zuq*u*q; 
        Zcomp(5) = Zuw*u*w;         Zcomp(6) = Zvp*v*p; 
        Zcomp(7) = m*(u*q - v*p + Zgp*(p*p + q*q) - Xgp*p*r - Ygp*q*r);
        Zcomp(8) = F1(3) + F2(3) + F3(3) + F4(3) ; % + bodyLift ;
    end

    function Kcomp = roll()
        Kcomp(1) = -(Ygp*Wp-Ybp*Bp)*cos(theta)*cos(phi) - (Zgp*Wp-Zbp*Bp)*cos(theta)*sin(phi);
        Kcomp(2) = Kpp*p*abs(p);
        Kcomp(3) = -(Izz-Iyy)*q*r - (m*Zgp)*w*p + (m*Zgp)*u*r;
        Kcomp(4) = Kprop;
        Kcomp(5) = M1(1) + M2(1) + M3(1) + M4(1);
    end

    function Mcomp = pitch()
        Mcomp(1) = -(Zgp*Wp - Zbp*Bp)*s2 - (Xgp*Wp - Xbp*Bp)*c2*c1;
        Mcomp(2) = Mww*w*abs(w);	Mcomp(3) = Mqq*q*abs(q);
    	Mcomp(4) = Muw*u*w;         Mcomp(5) = Muq*u*q; 
        Mcomp(6) = Mrp*p*r;         Mcomp(7) = (Izz - Ixx)*p*r;
        Mcomp(8) = -m*(Zgp*(w*q - v*r) + Xgp*(u*q - v*p));
        Mcomp(9) = M1(2) + M2(2) + M3(2) + M4(2); % + bodyMoment
    end

    function Ncomp = yaw()
        Ncomp(1) = (Ygp*Wp - Ybp*Bp)*s2 + (Xgp*Wp - Xbp*Bp)*c2*s1;
        Ncomp(2) = Nvv*v*abs(v);    Ncomp(3) = Nrr*r*abs(r); 
        Ncomp(4) = Nuv*u*v;         Ncomp(5) = Nur*u*r;
        Ncomp(6) = Npq*p*q;         Ncomp(7) = (Ixx - Iyy)*p*q;
        Ncomp(8) = - m*Xgp*(u*r - w*p) + m*Ygp*(w*q - v*r);
        Ncomp(9) = M1(3) + M2(3) + M3(3) + M4(3);
    end

end
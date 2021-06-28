clc;
close all;
clear 
% Assumptions.
% a. Wings are made of flexible membrane attached with
%    spar at leading edge and wing form is half elliptical.
% b. Only flapping would be induced by the power train
%    system with equal up/down flapping angles.
% c. The front spar will act as pivot for passive pitching
%    movement, caused by the aerodynamic/ inertial loads.
% d. The flow is assumed to remain attached.
% e. Both flapping and pitching movements are taken as
%    sinusoidal functions with certain amount of lag.
% f. Upstroke and down strokes have equal time duration.

% constants :-
g         = 9.81;
f         = 2.3277;          % frequency 
b         = 1.5;        % span
AR        = 6.8579;          % aspect ratio
C_root    = 0.2785;       % chord at root
phi       = pi/2;       % is the lag between pitching and flapping angle
theta0    = 30*pi/180;  % the maximum pitch angle
beta_max  = 30*pi/180;  % half of the stroke 60 degree
U         = 6;          % forward speed of ornithopter
delta     = 10*pi/180;   % flapping axis pitch angle or incidence angle 
        % Steady state aerodynamics - k=0
        % Quasi-steady aerodynamics - 0?k?0.05
        % Unsteady aerodynamics - k>0.05 [k>0.2 is considered highly unsteady]
k         =0.2;         % reduced frequency
rho       =1.225;
mu        =1.73e-5;
e         =0.8;         %the efficiency factor of the wing and is 0.8 for elliptical wing
Weight    =g*0.584;
Area      =pi/4*b*C_root;

% calculation :-
T=1/f;
m=100;                      %number of time steps
n=100;                      %number ot strips 
t=linspace(0,T,m);
r=linspace(0,b/2,n);
dr=b/2/n;
dt=T/m;
% j    index for time 
% i    index for space  
for i=1:n
    for j=1:m
        beta(1,j)      = beta_max*cos(2*pi*f*t(1,j));
        beta1(1,j)     = -2*pi*f*beta_max*sin(2*pi*f*t(1,j));
        beta2(1,j)     = -4*pi^2*f^2*beta_max*cos(2*pi*f*t(1,j));
        theta(i,j)     = 2*r(1,i)/b*theta0*cos(2*pi*f*t(1,j)+phi);
        theta1(i,j)    = -4*pi*f*r(1,i)/b*theta0*sin(2*pi*f*t(1,j));
        theta2(i,j)    = -8*pi^2*f^2*r(1,i)/b*theta0*cos(2*pi*f*t(1,j));
        
        c(1,i)         = 2*C_root/b*sqrt((b/2)^2-r(1,i)^2);
        Vx(i,j)        = U*cos(delta)+3/4*c(1,i)*theta1(i,j)*sin(theta(i,j));
        Vy(i,j)        = U*sin(delta)+(-r(1,i)*beta1(1,j)*cos(beta(1,j))+... 
                                    3/4*c(1,i)*theta1(i,j)*cos(theta(i,j)));
        Vrel(i,j)      = sqrt(Vx(i,j)^2+Vy(i,j)^2);
        psi(i,j)       = atan(Vy(i,j)/Vx(i,j));
        alpha_eff(i,j) = theta(i,j)+psi(i,j);
        
        c1             = 0.5*AR/(2.32+AR);
        c2             = 0.181+0.772/AR;
        F              = 1-c1*k^2/(k^2+c2^2);
        G              = -c1*c2*k/(k^2+c2^2);
        Cc             = sqrt(F^2+G^2);
        C_cl(i,j)      = 2*pi*Cc*sin(alpha_eff(i,j));
        d_Lc(i,j)      = 1/2*rho*Vrel(i,j)^2*C_cl(i,j)*c(1,i)*dr;
        d_Nc(i,j)      = -rho/4*pi*c(1,i)^2*(theta1(i,j)*U+r(1,i)*beta2(1,j)*cos(theta(i,j))-...
                                              1/2*theta2(i,j))*dr;
        Re_ref(i,j)    = rho*Vrel(i,j)*c(1,i)/mu;
        C_f(i,j)       =0.445*(log10(Re_ref(i,j)))^-2.58;
        C_dp(i,j)      =k*C_f(i,j);
        C_di(i,j)      =1/(e*pi*AR)*C_cl(i,j)^2;
        d_Dp(i,j)      =1/2*rho*Vrel(i,j)^2*C_dp(i,j)* c(1,i)*dr;
        d_Di(i,j)      =1/2*rho*Vrel(i,j)^2*C_di(i,j)* c(1,i)*dr;
        d_D(i,j)       =d_Dp(i,j)+d_Di(i,j);
        d_Fver(i,j)    =d_Lc(i,j)*cos( psi(i,j))*cos(delta)+...
                        d_Nc(i,j)*cos(-theta(i,j))*cos(beta(1,j))*cos(delta);
        d_Fhor(i,j)    =d_Lc(i,j)*sin( psi(i,j))*cos(delta)+...
                        d_Nc(i,j)*sin(-theta(i,j))*cos(delta)-...
                        d_D(i,j)*cos(psi(i,j))*cos(delta);
        d_My(i,j)      =d_Fver(i,j)*r(1,i);
    end
end
L=zeros(1,m);
Th=zeros(1,m);
D=zeros(1,m);
M=zeros(1,m);
for j=1:m
    for i=1:n
        L(1,j)=L(1,j)+d_Fver(i,j);
        Th(1,j)=Th(1,j)+d_Fhor(i,j);
        D(1,j)=D(1,j)+d_D(i,j);
        M(1,j)=M(1,j)+d_My(i,j);
    end
end

%for average lift & thrust & drag & moment 
L_avg=0;
T_avg=0;
D_avg=0;
M_avg=0;
for j=1:m
   L_avg=L_avg+2*L(1,j)/m;
   T_avg=T_avg+2*Th(1,j)/m;
   D_avg=D_avg+2*D(1,j)/m;
   M_avg=M_avg+M(1,j)/m;
end


%plotting :-
figure
plot(t/T,L,'b-',t/T,Th,'g-',t/T,D,'r-','LineWidth',1.3);
xlabel('(t/T), Non dimensional time in one cycle');
ylabel('Forces (N)');
title('Lift, Thrust and Drag');
legend('Lift','Thrust','Drag');
grid on;
grid minor;

figure
plot(t/T,M,'b-',t/T,M_avg*ones(1,100),'r-','LineWidth',1.3);
xlabel('(t/T), Non dimensional time in one cycle');
ylabel('Moment (N.m)');
title('Moment on wing pivot');
legend('Moment(t)','Average Moment');
grid on;
grid minor;


% results:
Torque_av=44*g/100;     %N.m
WingLoading=L_avg/g/Area;
results = table(Weight,L_avg,T_avg,D_avg,M_avg,Torque_av,WingLoading,...
    'RowNames',{'Results'})

% Although there are endless possibilities of varying the
% inputs and getting the results in different forms but effect of
% only few variable design parameters has been presented
% here for fixed size and geometry of wings for an
% ornithopter. It can be concluded from this study that:-
%   a. Lift of an ornithopter is most influenced by the
%      incidence angle and forward speed but least affected
%      by flapping frequency.
%   b. Thrust of an ornithopter is most affected by flapping
%      frequency and forward speed but least influenced by
%      incidence angle.
%   c. The drag force increases with increase in forward
%      speed, incidence angle, and flapping frequency.
%   d. The increase in total flapping angle, increases all the
%      forces but the effect is marked on drag force.

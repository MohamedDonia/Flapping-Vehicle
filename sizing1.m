clc;
clear all;
close all;
g=9.81;
rho=1.225;
% model 1 for sizing 
m1=0.584;
b1=2.24 * m1^0.53;
S1=0.69 * m1^1.04;
c1=4/pi*S1/b1;
AR1=4*b1/pi/c1;
f1=1.32 * m1^-0.6;
Q1=17.3/g * m1^0.02;



% model 2 for sizing 
m2=0.584;
Q2=1.78;   % wing loading
S2=m2/Q2;
% choose span
b2=1.5;
c2=4/pi*S2/b2;
AR2=4*b2/pi/c2;
f2=1.08*(m2^(1/3)*g^(1/2)*b2^(-1)*S2^(-1/4)*rho^(-1/3));


% results:
Model={'model 1';'model 2'};
Mass={m1;m2};
Area={S1;S2};
Span={b1;b2};
chord={c1;c2};
AspectRatio={AR1;AR2};
Frequency={f1;f2};
WingLoading={Q1;Q2};
results = table(Mass,Area,Span,chord,AspectRatio,Frequency,WingLoading,...
    'RowNames',Model)

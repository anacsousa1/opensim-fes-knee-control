global flagSTIM
global n_cycles
global flagFAT_R
global controlTypeEXO
global controlTypeFES
global refVel
global PIDparam
global ESParam
global ILCParam

%% PID parameters
PIDparam = [0.15,0.06,0.03];

%% PID-ES parameters
ESParam.A = 0.148/10;          ESParam.omega = 16; %10 Hz
ESParam.phase = 0;             ESParam.K = 400;%500;
ESParam.RC =  0.0080;

%% PID-ILC param
ILCParam.alpha = 0.2;
ILCParam.beta = 1-ILCParam.alpha;
ILCParam.gama = 5*PIDparam(1);

%% BB params
flagSTIM = 3*0.0148;

%% RUN
controlTypeEXO = 'PID'; % type of the exo hip controller
flagFAT_R = 0;      % value of the fatigue, 0 = no fatigue
n_cycles = 5;       % number of cycles (stpes)
refVel = 01;        % which ref? 01 or 03? [0.1 m/s or 0.3 m/s]
controlTypeFES = 'Open-loop';   % type of the FES knee controller

gait_script;        % run

